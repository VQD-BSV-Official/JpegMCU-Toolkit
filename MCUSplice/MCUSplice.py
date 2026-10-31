"""
 * MCUSplice.py - decompile jpeg image into text form or vice-versa
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






#!/usr/bin/env python3
"""
mcus - chèn / xoá MCU trong file JPEG (baseline/extended Huffman)
mà KHÔNG đổi cấu trúc JPEG (header, bảng DQT/DHT, kích thước, số MCU, DRI giữ nguyên).

Cách dùng:
  MCUSplice.py a.jpg ins10                chèn 10 MCU tại MCU (0,0)
  MCUSplice.py a.jpg del20                xoá 20 MCU bắt đầu từ MCU (0,0)
  MCUSplice.py a.jpg ins20 @10,50         tại MCU x=10,y=50: chèn 20 MCU TRƯỚC MCU này
  MCUSplice.py a.jpg del20 @10,50         tại MCU x=10,y=50: xoá 20 MCU (tính từ MCU này)
  thêm -o out.jpg để chọn file xuất (mặc định: <tên>_out.jpg)

Quy ước để giữ nguyên cấu trúc:
  - Tổng số MCU không đổi. insert  => phần đuôi bị đẩy ra ngoài và bị cắt bỏ.
                           delete  => phần đuôi được bù bằng MCU trung tính.
  - MCU chèn vào / MCU bù: mọi block có DC diff = 0 và AC = EOB (phẳng, giữ mức DC trước đó).
  - Thứ tự MCU tính theo raster: index = y * số_MCU_mỗi_hàng + x.
  - Các block AC được copy nguyên bit; chỉ mã hoá lại DC diff (vì DC mã hoá vi phân).
  - Restart marker (RSTn) được dựng lại theo DRI cũ.
"""
import argparse
import os
import re
import struct
import sys


class JpegError(Exception):
    pass


# ----------------------------------------------------------------- Huffman
def build_huff(counts, symbols):
    enc, lookup = {}, [None] * 65536
    code = k = 0
    for ln in range(1, 17):
        for _ in range(counts[ln - 1]):
            sym = symbols[k]
            k += 1
            enc[sym] = (code, ln)
            start, span = code << (16 - ln), 1 << (16 - ln)
            if start + span > 65536:
                print("DHT is invalid?")
                raise JpegError("DHT is invalid?")
            lookup[start:start + span] = [(ln, sym)] * span
            code += 1
        code <<= 1
    return enc, lookup


# ------------------------------------------------------------------ parse
UNSUPPORTED_SOF = {0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
END_OF_SCAN = re.compile(rb'\xff[^\x00\xd0-\xd7\xff]')
RST_SPLIT = re.compile(rb'\xff[\xd0-\xd7]')


def parse_jpeg(data):
    if data[:2] != b'\xff\xd8':
        print("Missing FFD8 (SOI)")
        raise JpegError("Missing FFD8 (SOI)")

    
    pos, dri, sof, huff = 2, 0, None, {}
    while True:
        if pos + 2 > len(data) or data[pos] != 0xFF:
            raise JpegError(f"Marker error offset {pos}")
        m = data[pos + 1]
        if m == 0xFF:
            pos += 1
            continue
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            pos += 2
            continue


        if m == 0xD9:
            print("FFDA (SOS) not found")
            raise JpegError("FFDA (SOS) not found")
        ln = struct.unpack('>H', data[pos + 2:pos + 4])[0]
        seg = data[pos + 4:pos + 2 + ln]


        if m in UNSUPPORTED_SOF:
            raise JpegError("Chỉ hỗ trợ JPEG baseline/extended Huffman (không progressive/arithmetic/lossless)")
        if m in (0xC0, 0xC1):
            h, w, nf = struct.unpack('>HHB', seg[1:6])
            comps = [(seg[6 + 3 * i], seg[7 + 3 * i] >> 4, seg[7 + 3 * i] & 15) for i in range(nf)]
            sof = (w, h, comps)
        elif m == 0xC4:
            i = 0
            while i < len(seg):
                cls, tid = seg[i] >> 4, seg[i] & 15
                counts = list(seg[i + 1:i + 17])
                n = sum(counts)
                huff[(cls, tid)] = build_huff(counts, seg[i + 17:i + 17 + n])
                i += 17 + n
        elif m == 0xDD:
            dri = struct.unpack('>H', seg[:2])[0]
        elif m == 0xDA:


            if sof is None:
                raise JpegError("SOS xuất hiện trước SOF")
            scan_start = pos + 2 + ln
            sos = [(seg[1 + 2 * j], seg[2 + 2 * j] >> 4, seg[2 + 2 * j] & 15) for j in range(seg[0])]
            return sof, huff, dri, sos, scan_start
        pos += 2 + ln


# ----------------------------------------------------------------- decode
def get_bits(d, start, end):
    n = end - start
    if n <= 0:
        return 0
    b0, b1 = start >> 3, (end + 7) >> 3
    v = int.from_bytes(d[b0:b1], 'big') >> (b1 * 8 - end)
    return v & ((1 << n) - 1)


def decode_segment(buf, nmcu, layout, ncomp, dc_lk, ac_lk):
    """Trả về list MCU; mỗi MCU là list block (dc_tuyệt_đối, ac_bits, ac_nbits)."""
    d = buf.replace(b'\xff\x00', b'\xff') + b'\xff\xff\xff\xff'
    limit = (len(d) - 4) * 8
    pos = 0
    preds = [0] * ncomp
    out = []
    frm = int.from_bytes
    for _ in range(nmcu):
        mcu = []
        for comp in layout:
            dl, al = dc_lk[comp], ac_lk[comp]
            i = pos >> 3
            e = dl[(frm(d[i:i + 3], 'big') >> (8 - (pos & 7))) & 0xFFFF]
            if e is None:
                raise JpegError("Mã DC không hợp lệ (scan hỏng?)")
            pos += e[0]
            s = e[1]
            if s:
                i = pos >> 3
                b = (frm(d[i:i + 3], 'big') >> (24 - (pos & 7) - s)) & ((1 << s) - 1)
                pos += s
                if b < (1 << (s - 1)):
                    b -= (1 << s) - 1
                preds[comp] += b
            ac_start = pos
            k = 1
            while k < 64:
                i = pos >> 3
                e = al[(frm(d[i:i + 3], 'big') >> (8 - (pos & 7))) & 0xFFFF]
                if e is None:
                    raise JpegError("Mã AC không hợp lệ (scan hỏng?)")
                pos += e[0]
                rs = e[1]
                s = rs & 15
                if s == 0:
                    if rs == 0xF0:
                        k += 16
                        continue
                    break
                pos += s
                k += (rs >> 4) + 1
            mcu.append((preds[comp], get_bits(d, ac_start, pos), pos - ac_start))
        if pos > limit:
            raise JpegError("Scan bị cắt cụt hoặc hỏng")
        out.append(mcu)
    return out


# ----------------------------------------------------------------- encode
class BitWriter:
    def __init__(self):
        self.buf, self.acc, self.n = bytearray(), 0, 0

    def put(self, v, n):
        if n == 0:
            return
        self.acc = (self.acc << n) | v
        self.n += n
        if self.n >= 8:
            nb, r = self.n >> 3, self.n & 7
            self.buf += (self.acc >> r).to_bytes(nb, 'big')
            self.acc &= (1 << r) - 1
            self.n = r

    def finish(self):
        if self.n:
            pad = 8 - self.n
            self.put((1 << pad) - 1, pad)
        return bytes(self.buf)


def put_dc(w, enc, diff):
    s = abs(diff).bit_length()
    if s not in enc:
        raise JpegError(f"Bảng Huffman DC của file không có mã cho category {s} "
                        "(bảng tối ưu hoá) - không thể mã hoá lại DC")
    w.put(*enc[s])
    if s:
        w.put(diff if diff > 0 else diff + (1 << s) - 1, s)


def rebuild_scan(mcus, ri, layout, ncomp, dc_enc, ac_enc):
    total = len(mcus)
    seg_size = ri or total
    nseg = -(-total // seg_size)
    try:
        eob = [ac_enc[c][0x00] for c in range(ncomp)]
    except KeyError:
        raise JpegError("Bảng Huffman AC không có mã EOB (0x00)")
    out = bytearray()
    for si in range(nseg):
        w = BitWriter()
        preds = [0] * ncomp
        for mcu in mcus[si * seg_size:(si + 1) * seg_size]:
            for bi, comp in enumerate(layout):
                if mcu is None:                       # MCU trung tính
                    put_dc(w, dc_enc[comp], 0)
                    w.put(*eob[comp])
                else:
                    dc, acv, acn = mcu[bi]
                    put_dc(w, dc_enc[comp], dc - preds[comp])
                    preds[comp] = dc
                    w.put(acv, acn)
        out += w.finish().replace(b'\xff', b'\xff\x00')
        if si != nseg - 1:
            out += bytes([0xFF, 0xD0 + (si & 7)])
    return bytes(out)


# ------------------------------------------------------------------- main
def process(data, x, y, action, n):
    (W, H, comps), huff, dri, sos, scan_start = parse_jpeg(data)
    nf = len(comps)
    if len(sos) != nf:
        raise JpegError("Scan không interleaved (nhiều scan) - không hỗ trợ")

    idx_of = {c[0]: i for i, c in enumerate(comps)}
    try:
        order = [idx_of[s[0]] for s in sos]
    except KeyError:
        raise JpegError("SOS tham chiếu component không tồn tại")

    # bảng Huffman theo component
    try:
        dc_enc = [None] * nf; dc_lk = [None] * nf
        ac_enc = [None] * nf; ac_lk = [None] * nf
        for (cid, td, ta) in sos:
            c = idx_of[cid]
            dc_enc[c], dc_lk[c] = huff[(0, td)]
            ac_enc[c], ac_lk[c] = huff[(1, ta)]
    except KeyError:
        raise JpegError("Thiếu bảng Huffman cho scan")

    hmax = max(c[1] for c in comps)
    vmax = max(c[2] for c in comps)
    if nf == 1:
        layout = [0]
        mx, my = -(-W // 8), -(-H // 8)
    else:
        layout = []
        for ci in order:                                # thứ tự component trong scan
            layout += [ci] * (comps[ci][1] * comps[ci][2])
        mx, my = -(-W // (8 * hmax)), -(-H // (8 * vmax))
    total = mx * my
    names = ['Y', 'Cb', 'Cr'] if nf == 3 else [f'C{i}' for i in range(nf)]
    print(f"{W}x{H} | MCU {mx}x{my} = {total} MCU | MCU = {[names[c] for c in layout]}"
          f" | DRI={dri}")

    if not (0 <= x < mx and 0 <= y < my):
        raise JpegError(f"MCU ({x},{y}) ngoài phạm vi (x<{mx}, y<{my})")
    idx = y * mx + x

    # tách scan
    m = END_OF_SCAN.search(data, scan_start)
    end = m.start() if m else len(data)
    if m and data[end + 1] != 0xD9:
        raise JpegError("Có nhiều scan - không hỗ trợ")
    raw, tail = data[scan_start:end], data[end:]
    parts = RST_SPLIT.split(raw)
    seg_size = dri or total
    nseg = -(-total // seg_size)
    if len(parts) != nseg:
        raise JpegError(f"Số đoạn RST ({len(parts)}) không khớp kỳ vọng ({nseg}) - file hỏng?")

    mcus = []
    for si, buf in enumerate(parts):
        cnt = min(seg_size, total - si * seg_size)
        mcus += decode_segment(buf, cnt, layout, nf, dc_lk, ac_lk)

    if action == 'insert':
        new = (mcus[:idx] + [None] * n + mcus[idx:])[:total]
        print(f"Insert {n} MCU ~ MCU ({x},{y}) [index {idx}]")
    else:
        start = idx
        if start >= total:
            raise JpegError("Không còn MCU nào để xoá")
        n = min(n, total - start)
        new = mcus[:start] + mcus[start + n:] + [None] * n
        print(f"Xoá {n} MCU từ index {start}; bù {n} MCU trung tính ở cuối")

    new_scan = rebuild_scan(new, dri, layout, nf, dc_enc, ac_enc)
    return data[:scan_start] + new_scan + tail


OP_RE = re.compile(r'^(ins|del)(\d*)$', re.I)
POS_RE = re.compile(r'^@(\d+),(\d+)$')


def main():
    ap = argparse.ArgumentParser(description="Insert/Delete MCU in JPEG, keep the structure",
                                 usage="%(prog)s input.jpg {ins|del}N [@x,y] [-o out.jpg]")
    ap.add_argument('input')
    ap.add_argument('op', help="insN or delN (N = number MCU, default 1)")
    ap.add_argument('pos', nargs='?', default='@0,0', help="@x,y (default @0,0)")
    ap.add_argument('-o', '--output')
    a = ap.parse_intermixed_args()

    mo, mp = OP_RE.match(a.op), POS_RE.match(a.pos)
    if not mo or not mp:
        ap.error("Demo: input.jpg ins10 or del10 [@x,y] [-o out.jpg]")

        
    action = 'insert' if mo.group(1).lower() == 'ins' else 'delete'
    n = int(mo.group(2) or 1)
    x, y = int(mp.group(1)), int(mp.group(2))

    data = open(a.input, 'rb').read()
    try:
        out = process(data, x, y, action, n)
    except JpegError as e:
        sys.exit(f"Lỗi: {e}")
    path = a.output or '{}_out{}'.format(*os.path.splitext(a.input))
    open(path, 'wb').write(out)
    print("Đã ghi:", path)


if __name__ == '__main__':
    main()
