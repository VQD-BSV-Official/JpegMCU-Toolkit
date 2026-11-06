#!/usr/bin/env python3
"""
 * MCUDecode_v2.py - decode JPEG Huffman scan data and log the bit offset of each MCU
 * Copyright (C) 2026 Vũ Quang Đại
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111 USA
 * or see <http://www.gnu.org/licenses/>
 """



import sys
import os
import math


class JpegError(Exception):
    pass


class HuffError(Exception):
    def __init__(self, msg, cls, tid):
        super().__init__(msg)
        self.msg, self.cls, self.tid = msg, cls, tid


class MarkerHit(Exception):
    def __init__(self, pos, marker):
        super().__init__("marker")
        self.pos, self.marker = pos, marker


class EndOfData(Exception):
    pass


class Huff:
    def __init__(self, counts, symbols):
        self.mincode = [0] * 17
        self.maxcode = [-1] * 17
        self.valptr = [0] * 17
        self.symbols = symbols
        code = 0
        k = 0
        for l in range(1, 17):
            c = counts[l - 1]
            self.valptr[l] = k
            self.mincode[l] = code
            code += c
            k += c
            self.maxcode[l] = code - 1 if c else -1
            code <<= 1


class BitReader:
    def __init__(self, data, pos, end):
        self.data = data
        self.pos = pos      # offset byte hiện tại trong file
        self.bit = 0        # bit hiện tại trong byte (0 = MSB)
        self.end = end

    def get_bit(self):
        if self.pos >= self.end:
            raise EndOfData()
        b = self.data[self.pos]
        stuffed = False
        if b == 0xFF:
            if self.pos + 1 >= self.end:
                raise EndOfData()
            n = self.data[self.pos + 1]
            if n != 0x00:
                raise MarkerHit(self.pos, n)
            stuffed = True
        v = (b >> (7 - self.bit)) & 1
        self.bit += 1
        if self.bit == 8:
            self.bit = 0
            self.pos += 2 if stuffed else 1
        return v

    def receive(self, n):
        v = 0
        for _ in range(n):
            v = (v << 1) | self.get_bit()
        return v

    def align(self):
        if self.bit:
            b = self.data[self.pos]
            self.pos += 2 if (b == 0xFF and self.pos + 1 < self.end
                              and self.data[self.pos + 1] == 0) else 1
            self.bit = 0

    def decode_huff(self, h, cls, tid):
        code = 0
        for l in range(1, 17):
            code = (code << 1) | self.get_bit()
            if h.maxcode[l] >= 0 and h.mincode[l] <= code <= h.maxcode[l]:
                return h.symbols[h.valptr[l] + code - h.mincode[l]]
        raise HuffError("Huffman error", cls, tid)


def extend(v, n):
    return v - ((1 << n) - 1) if v < (1 << (n - 1)) else v


def decode_block(r, dc, ac, td, ta, pred):
    s = r.decode_huff(dc, "DC", td)
    if s > 15:
        raise HuffError("Huffman error", "DC", td)
    diff = extend(r.receive(s), s) if s else 0
    pred += diff
    k = 1
    while k < 64:
        rs = r.decode_huff(ac, "AC", ta)
        run, size = rs >> 4, rs & 15
        if size == 0:
            if run == 15:
                k += 16
                if k > 64:
                    raise HuffError("Coefficient overflow", "AC", ta)
                continue
            break
        k += run
        if k > 63:
            raise HuffError("Coefficient overflow", "AC", ta)
        r.receive(size)
        k += 1
    return pred


def decode_scan(data, pos, end, frame, scan, tables, dri, out):
    width, height, fcomps = frame
    hmax = max(c[1] for c in fcomps.values())
    vmax = max(c[2] for c in fcomps.values())

    blocks = []  # (chỉ số comp trong scan, nhãn)
    for ci, (cid, h, v, td, ta) in enumerate(scan):
        n = 1 if len(scan) == 1 else h * v
        blocks += [(ci, "Y" if ci == 0 else "C")] * n

    if len(scan) == 1:
        cid, h, v, _, _ = scan[0]
        cw = math.ceil(width * h / hmax)
        ch = math.ceil(height * v / vmax)
        mcu_w = mcu_h = 8
        mcux, mcuy = math.ceil(cw / 8), math.ceil(ch / 8)
    else:
        mcu_w, mcu_h = 8 * hmax, 8 * vmax
        mcux, mcuy = math.ceil(width / mcu_w), math.ceil(height / mcu_h)
    total = mcux * mcuy

    out.write(f"// {width}x{height} {len(scan)} components:\n")
    for cid, h, v, td, ta in scan:
        out.write(f"// ID:{cid} [{h}{v}] Dest:{td}\n")
    out.write(f"// MCU: {''.join(b[1] for b in blocks)} ({mcu_w}x{mcu_h} pixel)\n")
    out.write(f"//[{mcux}x{mcuy}={total} MCU]\n\n\n")

    # Kiểm tra bảng Huffman
    for cid, h, v, td, ta in scan:
        if ("DC", td) not in tables or ("AC", ta) not in tables:
            raise JpegError(f"Thiếu bảng Huffman cho component {cid}")

    r = BitReader(data, pos, end)
    preds = [0] * len(scan)

    for n in range(total):
        # Restart interval
        if dri and n and n % dri == 0:
            r.align()
            if (r.pos + 1 < end and data[r.pos] == 0xFF
                    and 0xD0 <= data[r.pos + 1] <= 0xD7):
                r.pos += 2
            else:
                # tìm RSTn kế tiếp để đồng bộ lại
                p = r.pos
                while p + 1 < end and not (data[p] == 0xFF and 0xD0 <= data[p + 1] <= 0xD7):
                    p += 1
                out.write(f"*** RST marker missing at {r.pos}\n")
                if p + 1 >= end:
                    return
                r.pos = p + 2
            r.bit = 0
            preds = [0] * len(scan)

        x, y = n % mcux, n // mcux
        out.write(f"MCU {n} ({x},{y}) ({r.pos}.{r.bit})\n")

        for bi, (ci, label) in enumerate(blocks):
            cid, h, v, td, ta = scan[ci]
            off = f"{r.pos}.{r.bit}"
            try:
                preds[ci] = decode_block(r, tables[("DC", td)], tables[("AC", ta)],
                                         td, ta, preds[ci])
            except HuffError as e:
                out.write(f"*** [{label}@{off}] {e.msg} -> {e.cls}:{e.tid} #{bi}\n")
            except MarkerHit as m:
                out.write(f"*** [{label}@{off}] Marker FF{m.marker:02X} at {m.pos} -> stop\n")
                return
            except EndOfData:
                out.write(f"*** [{label}@{off}] End of data -> stop\n")
                return


def parse_jpeg(data, start, end, log_path, find_thumb=True):
    """Parse 1 JPEG nằm trong data[start:end]. Trả về (thumb_start, thumb_end) hoặc None."""
    if data[start:start + 2] != b"\xff\xd8":
        raise JpegError("Không thấy SOI (FFD8)")
    i = start + 2
    tables, frame, dri, thumb = {}, None, 0, None

    while i + 4 <= end:
        if data[i] != 0xFF:
            i += 1
            continue
        m = data[i + 1]
        if m == 0xFF:
            i += 1
            continue
        if m in (0x00, 0x01) or 0xD0 <= m <= 0xD8:
            i += 2
            continue
        if m == 0xD9:
            break
        ln = (data[i + 2] << 8) | data[i + 3]
        seg = data[i + 4:i + 2 + ln]

        if m == 0xE1 and find_thumb and thumb is None and seg[:6] == b"Exif\x00\x00":
            idx = seg.find(b"\xff\xd8\xff", 6)
            if idx >= 0:
                thumb = (i + 4 + idx, i + 2 + ln)
        elif m == 0xC4:  # DHT
            p = 0
            while p + 17 <= len(seg):
                tc, th = seg[p] >> 4, seg[p] & 15
                counts = list(seg[p + 1:p + 17])
                n = sum(counts)
                syms = list(seg[p + 17:p + 17 + n])
                tables[("DC" if tc == 0 else "AC", th)] = Huff(counts, syms)
                p += 17 + n
        elif m in (0xC0, 0xC1):  # SOF0/SOF1
            h_, w_, nc = (seg[1] << 8) | seg[2], (seg[3] << 8) | seg[4], seg[5]
            comps = {}
            for k in range(nc):
                cid, hv, tq = seg[6 + 3 * k:9 + 3 * k]
                comps[cid] = (cid, hv >> 4, hv & 15, tq)
            frame = (w_, h_, comps)
        elif m in (0xC2, 0xC3) or 0xC5 <= m <= 0xCF and m not in (0xC8, 0xCC):
            raise JpegError(f"SOF 0xFF{m:02X} chưa hỗ trợ (chỉ baseline/extended sequential Huffman)")
        elif m == 0xDD:
            dri = (seg[0] << 8) | seg[1]
        elif m == 0xDA:  # SOS
            if frame is None:
                raise JpegError("SOS trước SOF")
            ns = seg[0]
            scan = []
            for k in range(ns):
                cs, t = seg[1 + 2 * k], seg[2 + 2 * k]
                _, h, v, _ = frame[2][cs]
                scan.append((cs, h, v, t >> 4, t & 15))
            with open(log_path, "w", encoding="utf-8") as out:
                decode_scan(data, i + 2 + ln, end, frame, scan, tables, dri, out)
            return thumb
        i += 2 + ln
    raise JpegError("Không tìm thấy SOS")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    path = sys.argv[1]
    outdir = sys.argv[2] if len(sys.argv) > 2 else (os.path.dirname(os.path.abspath(path)))
    with open(path, "rb") as f:
        data = f.read()

    main_log = os.path.join(outdir, "Log_MCUs.txt")
    thumb = parse_jpeg(data, 0, len(data), main_log, True)
    print("Đã ghi", main_log)

    if thumb:
        thumb_log = os.path.join(outdir, "Log_MCUs_Thumb.txt")
        try:
            parse_jpeg(data, thumb[0], thumb[1], thumb_log, False)
            print("Đã ghi", thumb_log)
        except JpegError as e:
            print("Thumbnail lỗi:", e)


if __name__ == "__main__":
    main()