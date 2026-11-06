"""
 * MCUSplice.py - insert/delete MCUs in a JPEG scan without changing the JPEG structure
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
MCUSplice.py - chèn / xoá MCU trong JPEG (baseline/extended Huffman), giữ nguyên cấu trúc
(header, DQT/DHT, kích thước, số MCU, DRI).

Dùng: MCUSplice.py in.jpg [out.jpg] [{ins|del}N][@x,y][@x2,y2] [màu]     (out mặc định: <tên>_out.jpg)
  ins10                chèn 10 MCU tại (0,0)
  del20@10,50          xoá 20 MCU từ MCU x=10,y=50
  ins20@10,50          chèn 20 MCU trước MCU (10,50)
  del@127,10@142,12    xoá từ MCU (127,10) đến (142,12), gồm cả MCU cuối


  Chỉnh màu: y30 cb-20 cr15 = cộng vào DC (mức pixel); ycc128,0,0 = đặt tuyệt đối (`_` = bỏ qua kênh)
    ins: chỉnh N MCU vừa chèn | del: chỉnh N MCU sau vùng xoá | không ins/del: chỉnh từ @x,y (mặc định 0,0) đến hết ảnh

Log_MCUs.txt (cùng thư mục file vào): ins/del không kèm màu cắt bit theo log; chưa có/không khớp thì tự chạy MCUDecode.py.
  DC mã hoá vi phân nên MCU sau điểm cắt lệch màu đến RST kế tiếp (hết ảnh nếu DRI=0).
  Có chỉnh màu thì giải mã + mã hoá lại DC (AC copy nguyên bit).

Quy ước: MCU đánh số raster (index = y * MCU_mỗi_hàng + x); tổng số MCU không đổi.
  ins cắt phần đuôi, del bù đuôi bằng MCU trung tính (DC diff = 0, AC = EOB); RSTn dựng lại theo DRI cũ.
"""
import argparse
import os
import re
import struct
import subprocess
import sys
from bisect import bisect_left


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
                raise JpegError("Invalid DHT")
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
        raise JpegError("Missing SOI (FFD8)")

    
    pos, dri, sof, huff, q0 = 2, 0, None, {}, {}
    while True:
        if pos + 2 > len(data) or data[pos] != 0xFF:
            raise JpegError(f"Bad marker at offset {pos}")
        m = data[pos + 1]
        if m == 0xFF:
            pos += 1
            continue
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            pos += 2
            continue


        if m == 0xD9:
            raise JpegError("SOS (FFDA) not found")
        ln = struct.unpack('>H', data[pos + 2:pos + 4])[0]
        seg = data[pos + 4:pos + 2 + ln]


        if m in UNSUPPORTED_SOF:
            raise JpegError("Only baseline/extended Huffman JPEG is supported")
        if m in (0xC0, 0xC1):
            h, w, nf = struct.unpack('>HHB', seg[1:6])
            comps = [(seg[6 + 3 * i], seg[7 + 3 * i] >> 4, seg[7 + 3 * i] & 15, seg[8 + 3 * i]) for i in range(nf)]
            sof = (w, h, comps)
        elif m == 0xDB:
            i = 0
            while i < len(seg):
                pq, tq = seg[i] >> 4, seg[i] & 15
                q0[tq] = int.from_bytes(seg[i + 1:i + 2 + pq], 'big')
                i += 1 + 64 * (pq + 1)
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
                raise JpegError("SOS before SOF")
            scan_start = pos + 2 + ln
            sos = [(seg[1 + 2 * j], seg[2 + 2 * j] >> 4, seg[2 + 2 * j] & 15) for j in range(seg[0])]
            return sof, huff, dri, sos, scan_start, q0
        pos += 2 + ln


# ----------------------------------------------------------------- decode
def get_bits(d, start, end):
    n = end - start
    if n <= 0:
        return 0
    b0, b1 = start >> 3, (end + 7) >> 3
    v = int.from_bytes(d[b0:b1], 'big') >> (b1 * 8 - end)
    return v & ((1 << n) - 1)


def decode_segment(buf, nmcu, layout, ncomp, dc_lk, ac_lk, out, base):
    """Ghi MCU giải mã được vào out (giữ nguyên phần đã giải mã khi gặp lỗi); mỗi MCU là list block (dc_tuyệt_đối, ac_bits, ac_nbits)."""
    d = buf.replace(b'\xff\x00', b'\xff') + b'\xff\xff\xff\xff'
    limit = (len(d) - 4) * 8
    pos = 0
    preds = [0] * ncomp
    frm = int.from_bytes
    for mi in range(nmcu):
        mcu = []
        for comp in layout:
            dl, al = dc_lk[comp], ac_lk[comp]
            i = pos >> 3
            e = dl[(frm(d[i:i + 3], 'big') >> (8 - (pos & 7))) & 0xFFFF]
            if e is None:
                raise JpegError(f"bad DC code at MCU #{base + mi}")
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
                    raise JpegError(f"bad AC code at MCU #{base + mi}")
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
            raise JpegError(f"scan truncated at MCU #{base + mi}")
        out.append(mcu)


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
        raise JpegError(f"DC Huffman table has no code for category {s}; cannot re-encode DC")
    w.put(*enc[s])
    if s:
        w.put(diff if diff > 0 else diff + (1 << s) - 1, s)


def rebuild_scan(mcus, ri, layout, ncomp, dc_enc, eob):
    total = len(mcus)
    seg_size = ri or total
    nseg = -(-total // seg_size)
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


# -------------------------------------------------------- log-based splice
LOG_RE = re.compile(r'^MCU (\d+) \(\d+,\d+\) \((\d+)\.(\d)\)', re.M)


def read_log(path, hdr, first):
    """{mcu: bit offset trong file} nếu log khớp ảnh (header + MCU đầu), ngược lại None."""
    try:
        txt = open(path, encoding='utf-8').read()
    except OSError:
        return None
    offs = {int(a): int(b) * 8 + int(c) for a, b, c in LOG_RE.findall(txt)}
    return offs if hdr in txt and offs.get(0) == first else None


def get_offs(src, mx, my, total, scan_start):
    here = os.path.dirname(os.path.abspath(__file__))
    log = os.path.join(os.path.dirname(os.path.abspath(src)), 'Log_MCUs.txt')
    args = (log, f'//[{mx}x{my}={total} MCU]', scan_start * 8)
    offs = read_log(*args)
    if offs is None:
        print("No valid Log_MCUs.txt, running MCUDecode.py")
        if subprocess.run([sys.executable, os.path.join(here, 'MCUDecode.py'), src], capture_output=True).returncode:
            raise JpegError("MCUDecode.py failed")
        if (offs := read_log(*args)) is None:
            raise JpegError("Log_MCUs.txt does not match the file")
    return offs


def mcu_end(d, pos, layout, dc_lk, ac_lk):
    """Bit ngay sau MCU bắt đầu tại pos (cần cho MCU cuối đoạn: log không ghi điểm kết thúc)."""
    frm = int.from_bytes
    for c in layout:
        e = dc_lk[c][(frm(d[pos >> 3:(pos >> 3) + 3], 'big') >> (8 - (pos & 7))) & 0xFFFF]
        if e is None:
            raise JpegError("DC")
        pos += e[0] + e[1]
        k = 1
        while k < 64:
            e = ac_lk[c][(frm(d[pos >> 3:(pos >> 3) + 3], 'big') >> (8 - (pos & 7))) & 0xFFFF]
            if e is None:
                raise JpegError("AC")
            pos += e[0]
            rs = e[1]
            if rs & 15:
                pos += rs & 15
                k += (rs >> 4) + 1
            elif rs == 0xF0:
                k += 16
            else:
                break
    return pos


def to_chunks(parts, ps, seg, total, offs, layout, dc_lk, ac_lk):
    """Mỗi MCU -> (bits, nbits) cắt từ scan đã bỏ stuffing; MCU không có trong log / không cắt được -> None."""
    mcus = []
    for si, buf in enumerate(parts):
        d = buf.replace(b'\xff\x00', b'\xff') + b'\xff\xff\xff\xff'
        limit = (len(d) - 4) * 8
        F, j = [], buf.find(b'\xff\x00')              # vị trí FF00 -> đổi offset file sang offset đã bỏ stuffing
        while j >= 0:
            F.append(j)
            j = buf.find(b'\xff\x00', j + 2)
        lo, hi = si * seg, min((si + 1) * seg, total)
        st = []
        for m in range(lo, hi):
            o = offs.get(m)
            rel = (o >> 3) - ps[si] if o is not None else -1
            st.append(((rel - bisect_left(F, rel)) << 3) | (o & 7) if 0 <= rel <= len(buf) else None)
        st.append(None)
        for i in range(hi - lo):
            a, b = st[i], st[i + 1]
            if a is not None and b is None:
                try:
                    b = mcu_end(d, a, layout, dc_lk, ac_lk)
                except JpegError:
                    pass
            mcus.append((get_bits(d, a, b), b - a) if a is not None and b is not None and a < b <= limit else None)
    return mcus


def rebuild_chunks(chunks, ri, neutral):
    total = len(chunks)
    seg = ri or total
    out = bytearray()
    for si in range(-(-total // seg)):
        w = BitWriter()
        for ch in chunks[si * seg:(si + 1) * seg]:
            w.put(*(ch or neutral))
        out += w.finish().replace(b'\xff', b'\xff\x00')
        if (si + 1) * seg < total:
            out += bytes([0xFF, 0xD0 + (si & 7)])
    return bytes(out)


# ------------------------------------------------------------------- main
def process(data, x, y, action, n, adj, src, end=None):
    (W, H, comps), huff, dri, sos, scan_start, q0 = parse_jpeg(data)
    nf = len(comps)
    if len(sos) != nf:
        raise JpegError("Non-interleaved (multi-scan) JPEG is not supported")

    idx_of = {c[0]: i for i, c in enumerate(comps)}
    try:
        order = [idx_of[s[0]] for s in sos]
    except KeyError:
        raise JpegError("SOS references a missing component")

    # bảng Huffman theo component
    try:
        dc_enc = [None] * nf; dc_lk = [None] * nf
        ac_enc = [None] * nf; ac_lk = [None] * nf
        for (cid, td, ta) in sos:
            c = idx_of[cid]
            dc_enc[c], dc_lk[c] = huff[(0, td)]
            ac_enc[c], ac_lk[c] = huff[(1, ta)]
    except KeyError:
        raise JpegError("Missing Huffman table for scan")
    try:
        eob = [ac_enc[c][0] for c in range(nf)]
    except KeyError:
        raise JpegError("AC Huffman table has no EOB code")
    if adj and (max(adj) >= nf or any(comps[c][3] not in q0 for c in adj)):
        raise JpegError("Missing component or DQT table for color adjust")

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

    if not (0 <= x < mx and 0 <= y < my):
        raise JpegError(f"MCU ({x},{y}) out of range (x<{mx}, y<{my})")
    idx = y * mx + x
    if end:                                           # del@x1,y1@x2,y2: xoá đến hết MCU (x2,y2)
        if not (0 <= end[0] < mx and 0 <= end[1] < my):
            raise JpegError(f"MCU {end} out of range (x<{mx}, y<{my})")
        n = end[1] * mx + end[0] - idx + 1
        if n < 1:
            raise JpegError("End MCU is before start MCU")

    # tách scan
    m = END_OF_SCAN.search(data, scan_start)
    end = m.start() if m else len(data)
    if m and data[end + 1] != 0xD9:
        raise JpegError("Multi-scan JPEG is not supported")
    raw, tail = data[scan_start:end], data[end:]
    parts = RST_SPLIT.split(raw)
    seg_size = dri or total
    nseg = -(-total // seg_size)
    if len(parts) != nseg:
        raise JpegError(f"RST segments ({len(parts)}) != expected ({nseg}); corrupt file?")

    offs = get_offs(src, mx, my, total, scan_start) if action and not adj else None
    if offs is not None:                              # cắt bit theo log, không giải mã
        ps, p = [], scan_start
        for b in parts:
            ps.append(p)
            p += len(b) + 2
        mcus = to_chunks(parts, ps, seg_size, total, offs, layout, dc_lk, ac_lk)
        lost = mcus.count(None)
        if lost:
            print(f"Warning: {lost} MCU missing in log/unreadable -> neutral")
    else:
        mcus = []
        for si, buf in enumerate(parts):
            cnt = min(seg_size, total - si * seg_size)
            seg = []
            try:
                decode_segment(buf, cnt, layout, nf, dc_lk, ac_lk, seg, len(mcus))
            except JpegError as e:                    # scan hỏng: giữ phần giải mã được, phần mất -> MCU trung tính
                print(f"Warning: {e}; {cnt - len(seg)} MCU lost")
            mcus += seg + [None] * (cnt - len(seg))

    if action == 'insert':
        new = (mcus[:idx] + [None] * n + mcus[idx:])[:total]
        hi = min(idx + n, total)
        info = f"ins {n} @{x},{y}"
    elif action == 'delete':
        start = idx
        if start >= total:
            raise JpegError("No MCU left to delete")
        n = min(n, total - start)
        new = mcus[:start] + mcus[start + n:] + [None] * n
        hi = min(idx + n, total - n)              # bỏ phần MCU bù ở đuôi
        info = f"del {n} @{x},{y}"
    else:
        new, hi, info = mcus[:], total, f"@{x},{y}"

    sel = range(idx, hi)
    if adj:
        info += f" color x{len(sel)}"
        # V (mức pixel) -> DC lượng tử = 8·V/Q0; Y tuyệt đối tính so với mức 128
        adj = {c: (ab, round(8 * (v - 128 * (ab and c == 0)) / q0[comps[c][3]]))
               for c, (ab, v) in adj.items()}
        base = [0] * nf
        if idx:
            for c, blk in zip(layout, mcus[idx - 1]):
                base[c] = blk[0]

        def tweak(c, dc):
            ab, v = adj[c]
            return max(-2047, min(2047, v if ab else dc + v))

        for i in sel:
            blocks = new[i] or [(base[c], *eob[c]) for c in layout]
            new[i] = [(tweak(c, dc) if c in adj else dc, acv, acn)
                      for c, (dc, acv, acn) in zip(layout, blocks)]

    if offs is None:
        new_scan = rebuild_scan(new, dri, layout, nf, dc_enc, eob)
    else:
        if any(0 not in dc_enc[c] for c in layout):
            raise JpegError("DC Huffman table has no category 0; cannot build neutral MCU")
        neutral = (0, 0)
        for c in layout:
            for v, ln in (dc_enc[c][0], eob[c]):
                neutral = ((neutral[0] << ln) | v, neutral[1] + ln)
        new_scan = rebuild_chunks(new, dri, neutral)
    print(f"{W}x{H} MCU {mx}x{my}={total} DRI={dri} | {info}")
    return data[:scan_start] + new_scan + tail


ARG_RE = re.compile(r'^(?:(ins|del)(\d*))?(?:@(\d+),(\d+))?(?:@(\d+),(\d+))?$', re.I)
CH_RE = re.compile(r'^(y|cb|cr)([+-]?\d+)$', re.I)
YCC_RE = re.compile(r'^ycc(_|[+-]?\d+),(_|[+-]?\d+),(_|[+-]?\d+)$', re.I)
CH = {'y': 0, 'cb': 1, 'cr': 2}


def main():
    ap = argparse.ArgumentParser(description="Insert/delete MCU in a JPEG, keeping its structure",
                                 usage="%(prog)s in.jpg [out.jpg] [{ins|del}N][@x,y][@x2,y2] [yV] [cbV] [crV] [yccY,Cb,Cr]")
    ap.add_argument('args', nargs='+')
    files, action, n, x, y, adj, end = [], None, 1, 0, 0, {}, None
    for t in ap.parse_args().args:
        if t and (m := ARG_RE.match(t)):
            if m[1]:
                action, n = ('insert' if m[1].lower() == 'ins' else 'delete'), int(m[2] or 1)
            if m[3]:
                x, y = int(m[3]), int(m[4])
            if m[5]:
                if not (m[1] and m[1].lower() == 'del' and not m[2] and m[3]):
                    ap.error("Range syntax: del@x1,y1@x2,y2")
                end = (int(m[5]), int(m[6]))
        elif m := CH_RE.match(t):
            adj[CH[m[1].lower()]] = (False, int(m[2]))
        elif m := YCC_RE.match(t):
            adj.update({c: (True, int(v)) for c, v in enumerate(m.groups()) if v != '_'})
        else:
            files.append(t)
    if not (action or adj) or not 1 <= len(files) <= 2:
        ap.error("Demo: input.jpg [out.jpg] ins10@10,50 y30 cb-20 cr15")

    data = open(files[0], 'rb').read()
    try:
        out = process(data, x, y, action, n, adj, files[0], end)
    except JpegError as e:
        sys.exit(f"Error: {e}")
    path = files[1] if len(files) > 1 else '{}_out{}'.format(*os.path.splitext(files[0]))
    open(path, 'wb').write(out)
    print("Saved:", path)


if __name__ == '__main__':
    main()