#!/usr/bin/env python3
"""
 * MCUDecode.py - decode JPEG Huffman scan data and log the bit offset of each MCU
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
import re
import math
import time
from bisect import bisect_left


USAGE = """\
Usage: MCUDecode.py image.jpg [--adv]

Writes Log_MCUs.txt (main image) and Log_MCUs_Thumb.txt (EXIF thumbnail, if any) next to the image.
Offsets are <byte>.<bit> from the start of the file (bit 0 = MSB).
--adv: keep decoding MCUs until FFD9, ignoring the header size.
"""

PAD = bytes(512)                      # đệm 0 cuối dữ liệu để khỏi kiểm tra biên ở vòng lặp nóng
RST = [bytes((0xFF, 0xD0 + i)) for i in range(8)]
RST_RE = re.compile(rb"\xff[\xd0-\xd7]")


class JpegError(Exception):
    pass


def build_lut(counts, symbols, is_dc):
    """Bảng tra 16 bit -> kết quả (0 = mã không hợp lệ).
    DC: giá trị = codelen + size (tổng số bit cần bỏ qua).
    AC: giá trị = (codelen + size) << 8 | inc  (inc = số hệ số tiến lên, 0 = EOB)."""
    lut = [0] * 65536
    code = 0
    k = 0
    for l in range(1, 17):
        for _ in range(counts[l - 1]):
            if k >= len(symbols) or (code >> l):
                return lut
            sym = symbols[k]
            k += 1
            lo = code << (16 - l)
            hi = (code + 1) << (16 - l)
            if is_dc:
                if sym <= 15:
                    lut[lo:hi] = [l + sym] * (hi - lo)
            else:
                run, size = sym >> 4, sym & 15
                if size == 0:
                    inc = 16 if run == 15 else 0
                else:
                    inc = run + 1
                lut[lo:hi] = [((l + size) << 8) | inc] * (hi - lo)
            code += 1
        code <<= 1
    return lut


def decode_interval(data, p, q, n0, cnt, mcux, blocks, out, stop_at_end=False):
    """Decode tối đa cnt MCU (bắt đầu từ MCU n0) trong data[p:q]."""
    seg = data[p:q]
    U = seg.replace(b"\xff\x00", b"\xff")        # bỏ byte stuffing
    S = []                                        # vị trí (trong U) của các FF đã stuffing
    if len(U) != len(seg):
        c = 0
        j = seg.find(b"\xff\x00")
        while j >= 0:
            S.append(j - c)
            c += 1
            j = seg.find(b"\xff\x00", j + 2)
    limit = len(U) << 3
    U += PAD

    from_bytes = int.from_bytes
    bl = bisect_left
    append = out.append
    acc = 0
    nbits = 0
    pos = 0

    for n in range(n0, n0 + cnt):
        bp = (pos << 3) - nbits
        if stop_at_end and limit - bp < 8:      # chỉ còn bit đệm trước FFD9
            return "end", n, limit - bp
        u = bp >> 3
        append(f"MCU {n} ({n % mcux},{n // mcux}) ({p + u + bl(S, u)}.{bp & 7})")

        for ld, la, lab, td, ta, bi in blocks:
            bs_pos = pos
            bs_n = nbits
            err = 0

            # ---- DC ----
            if nbits < 32:
                acc = ((acc & ((1 << nbits) - 1)) << 32) | from_bytes(U[pos:pos + 4], "big")
                pos += 4
                nbits += 32
            e = ld[(acc >> (nbits - 16)) & 0xFFFF]
            if e:
                nbits -= e
                # ---- AC ----
                k = 1
                while k < 64:
                    if nbits < 32:
                        acc = ((acc & ((1 << nbits) - 1)) << 32) | from_bytes(U[pos:pos + 4], "big")
                        pos += 4
                        nbits += 32
                    e = la[(acc >> (nbits - 16)) & 0xFFFF]
                    if not e:
                        nbits -= 16
                        err = 2
                        break
                    nbits -= e >> 8
                    inc = e & 255
                    if not inc:
                        break
                    k += inc
                    if k > 64:
                        err = 3
                        break
            else:
                nbits -= 16
                err = 1

            over = (pos << 3) - nbits > limit
            if over or err:
                bpb = (bs_pos << 3) - bs_n
                ub = bpb >> 3
                o = f"{p + ub + bl(S, ub)}.{bpb & 7}"
                if over:
                    append(f"*** [{lab}@{o}] End of data -> stop")
                    return "eod", n + 1, 0
                if err == 1:
                    append(f"*** [{lab}@{o}] Huffman error -> DC:{td} #{bi}")
                elif err == 2:
                    append(f"*** [{lab}@{o}] Huffman error -> AC:{ta} #{bi}")
                else:
                    append(f"*** [{lab}@{o}] Coefficient overflow -> AC:{ta} #{bi}")

    return "done", n0 + cnt, limit - ((pos << 3) - nbits)


def decode_scan(data, a, end, frame, scan, tables, dri, log_path, adv=False):
    width, height, fcomps = frame
    hmax = max(c[1] for c in fcomps.values())
    vmax = max(c[2] for c in fcomps.values())

    for cid, h, v, td, ta in scan:
        if ("DC", td) not in tables or ("AC", ta) not in tables:
            raise JpegError(f"Missing Huffman table for component {cid}")

    blocks = []
    for ci, (cid, h, v, td, ta) in enumerate(scan):
        nb = 1 if len(scan) == 1 else h * v
        for _ in range(nb):
            blocks.append((tables[("DC", td)], tables[("AC", ta)],
                           "Y" if ci == 0 else "C", td, ta, len(blocks)))

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

    out = [f"// {width}x{height} {len(scan)} components:"]
    for cid, h, v, td, ta in scan:
        out.append(f"// ID:{cid} [{h}{v}] Dest:{td}")
    out.append(f"// MCU: {''.join(b[2] for b in blocks)} ({mcu_w}x{mcu_h} pixel)")
    out.append(f"//[{mcux}x{mcuy}={total} MCU]")
    out.append("")
    out.append("")

    # Vùng dữ liệu entropy: từ sau SOS đến EOI cuối cùng (nếu có)
    scan_end = data.rfind(b"\xff\xd9", a, end)
    if scan_end < 0:
        scan_end = end

    INF = 1 << 60
    total_eff = INF if adv else total
    ivl = dri if dri else INF
    n = 0
    p = a
    k = 0
    while n < total_eff:
        cnt = min(ivl, total_eff - n)
        found = False
        q = scan_end
        if dri:
            q = data.find(RST[k & 7], p, scan_end)
            if q < 0:
                mm = RST_RE.search(data, p, scan_end)
                q = mm.start() if mm else -1
                out.append(f"*** RST marker FF{0xD0 + (k & 7):02X} not found after {p}")
            if q < 0:
                q = scan_end
            else:
                found = True
        last = not found
        if adv and last:
            cnt = INF
        status, nn, rem = decode_interval(data, p, q, n, cnt, mcux, blocks, out, adv and last)
        if last:
            mk = "FFD9" if data[q:q + 2] == b"\xff\xd9" else "end of data"
            if adv:
                out.append(f"// END: {mk} at {q} - {nn} MCU (header: {total} MCU)")
            elif status == "done" and rem >= 8:
                out.append(f"// NOTE: còn {rem >> 3} byte trước {mk} (offset {q}) sau {total} MCU - dùng --adv")
            break
        n += cnt
        p = q + 2
        k += 1

    with open(log_path, "w", encoding="utf-8") as f:
        f.write("\n".join(out))
        f.write("\n")


def parse_jpeg(data, start, end, log_path, find_thumb=True, adv=False):
    """Parse 1 JPEG nằm trong data[start:end]. Trả về (thumb_start, thumb_end) hoặc None."""
    if data[start:start + 2] != b"\xff\xd8":
        raise JpegError("Missing SOI (FFD8)")
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
                is_dc = (tc == 0)
                tables[("DC" if is_dc else "AC", th)] = build_lut(counts, syms, is_dc)
                p += 17 + n
        elif m in (0xC0, 0xC1):  # SOF0/SOF1
            h_, w_, nc = (seg[1] << 8) | seg[2], (seg[3] << 8) | seg[4], seg[5]
            comps = {}
            for k in range(nc):
                cid, hv, tq = seg[6 + 3 * k:9 + 3 * k]
                comps[cid] = (cid, hv >> 4, hv & 15, tq)
            frame = (w_, h_, comps)
        elif m in (0xC2, 0xC3) or 0xC5 <= m <= 0xCF and m not in (0xC8, 0xCC):
            raise JpegError(f"SOF 0xFF{m:02X} not supported (baseline/extended sequential Huffman only)")
        elif m == 0xDD:
            dri = (seg[0] << 8) | seg[1]
        elif m == 0xDA:  # SOS
            if frame is None:
                raise JpegError("SOS before SOF")
            ns = seg[0]
            scan = []
            for k in range(ns):
                cs, t = seg[1 + 2 * k], seg[2 + 2 * k]
                _, h, v, _ = frame[2][cs]
                scan.append((cs, h, v, t >> 4, t & 15))
            decode_scan(data, i + 2 + ln, end, frame, scan, tables, dri, log_path, adv)
            return thumb
        i += 2 + ln
    raise JpegError("SOS not found")


def main():
    adv = "--adv" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--adv"]
    if not args:
        sys.exit(USAGE)
    with open(args[0], "rb") as f:
        data = f.read()
    d = os.path.dirname(os.path.abspath(args[0]))
    log, tlog = os.path.join(d, "Log_MCUs.txt"), os.path.join(d, "Log_MCUs_Thumb.txt")
    t0 = time.perf_counter()
    try:
        thumb = parse_jpeg(data, 0, len(data), log, True, adv)
        if thumb:
            parse_jpeg(data, *thumb, tlog, False, adv)
    except JpegError as e:
        sys.exit(f"Error: {e}")
    print(f"Wrote {log}{', ' + tlog if thumb else ''} ({time.perf_counter() - t0:.2f}s)")


if __name__ == "__main__":
    main()
