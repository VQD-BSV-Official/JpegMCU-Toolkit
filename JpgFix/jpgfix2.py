#!/usr/bin/env python3
# jpgfix2.py - sửa file JPEG baseline bị hỏng vùng đầu scan
# Copyright (C) 2026 Vũ Quang Đại
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; version 2 of the License.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
"""Module: fix(data, ref=None, off=None, win=256, skip=8) -> (bytes, info)
  data = bytes JPEG: header đúng (tới hết SOS) + phần scan còn nguyên (đã bỏ vùng bị mã hoá).
  ref  = ảnh tham chiếu (đường dẫn | file-like | PIL.Image); off = (dY,dCb,dCr); lỗi -> ValueError.
CLI: jpgfix2.py in.jpg out.jpg [ref=anh.jpg] [off=dY,dCb,dCr] [win=256] [skip=8]
Tìm MCU tốt đầu tiên, căn theo hàng MCU, cắt height (không còn phần xám). Baseline Huffman."""
import sys, re, struct, io
import numpy as np
from array import array

def huff(cnt, sym):
    dec, enc, code, k = [0] * 65536, {}, 0, 0
    for L in range(1, 17):
        for _ in range(cnt[L - 1]):
            s = sym[k]; k += 1
            enc[s] = (code, L)
            lo, n = code << (16 - L), 1 << (16 - L)
            dec[lo:lo + n] = [(L << 8) | s] * n
            code += 1
        code <<= 1
    return dec, enc

def parse(b):
    i, T, Q, dri = 2, {}, {}, 0
    while True:
        m, L = b[i + 1], struct.unpack('>H', b[i + 2:i + 4])[0]
        s = b[i + 4:i + 2 + L]
        if m == 0xC4:
            while s:
                n = sum(s[1:17]); T[s[0] >> 4, s[0] & 15] = huff(s[1:17], s[17:17 + n]); s = s[17 + n:]
        elif m == 0xDB:
            while s:
                pr = s[0] >> 4; Q[s[0] & 15] = s[1] << 8 | s[2] if pr else s[1]; s = s[65 + 64 * pr:]
        elif m == 0xC0:
            H, W, nf = struct.unpack('>HHB', s[1:6]); sofp = i
            cs = [(s[6 + 3 * k], s[7 + 3 * k] >> 4, s[7 + 3 * k] & 15, s[8 + 3 * k]) for k in range(nf)]
        elif m == 0xC2: raise ValueError('progressive không hỗ trợ')
        elif m == 0xDD: dri = struct.unpack('>H', s)[0]
        elif m == 0xDA:
            return i + 2 + L, T, W, H, cs, [s[2 + 2 * k] for k in range(s[0])], dri, sofp, Q
        i += 2 + L

def pack(v, n):  # pad bit 1 + byte stuffing
    p = -n % 8
    return ((v << p) | ((1 << p) - 1)).to_bytes((n + p) // 8, 'big').replace(b'\xff', b'\xff\x00')

def cat(a, b): return (a[0] << b[1]) | b[0], a[1] + b[1]

def fix(b, ref=None, off=None, win=256, skip=8):
    b = bytes(b)
    se, T, W, H, cs, sel, dri, sofp, Q = parse(b)
    hdr = b[:se]
    seq = [k for k, c in enumerate(cs) for _ in range(c[1] * c[2])]
    vm = max(c[2] for c in cs); mx = -(-W // (8 * max(c[1] for c in cs)))
    tot = mx * -(-H // (8 * vm))
    dc = [T[0, s >> 4] for s in sel]; ac = [T[1, s & 15] for s in sel]

    t = b[se:]; e = t.find(b'\xff\xd9')  # EOI thật đầu tiên, bỏ trailer phía sau
    t = t[:e + 2] if e >= 0 else t + b'\xff\xd9'

    def out(body, rows):  # height mới = bỏ các hàng MCU đã mất -> không còn vùng xám
        return hdr[:sofp + 5] + struct.pack('>H', H - rows * 8 * vm) + hdr[sofp + 7:] + body

    if dri:  # có RST: DC tự reset, đồng bộ theo marker
        ms = [m.start() for m in re.finditer(rb'\xff[\xd0-\xd7]', t)]
        j0 = -(-tot // dri) - 1 - len(ms)  # chỉ số interval của marker đầu tiên còn nguyên
        sk = next(x for x in range(len(ms)) if (j0 + 1 + x) * dri % mx == 0)
        n = [0]
        def rn(m): n[0] += 1; return bytes([255, 0xD0 + (n[0] - 1) % 8])  # đánh số lại RST từ 0
        rows = (j0 + 1 + sk) * dri // mx
        return (out(re.sub(rb'\xff[\xd0-\xd7]', rn, t[ms[sk] + 2:]), rows),
                f'RST: cắt {rows} hàng MCU, height {H} -> {H - rows * 8 * vm}')

    d = t[:-2].replace(b'\xff\x00', b'\xff')
    E = len(d) * 8; BIG = int.from_bytes(d, 'big')
    x = np.frombuffer(d + b'\0\0', 'u1').astype('u4')
    w = array('I')  # w[i] = 24 bit từ byte i -> peek16 = shift + mask
    w.frombytes(((x[:-2] << 16) | (x[1:-1] << 8) | x[2:]).tobytes()); w.extend([0] * 300)

    def conv(e):  # AC: (bit tiến << 8) | số hệ số tiến; 255=EOB, 0=sai
        L, rs = divmod(e, 256)
        if not L: return 0
        if rs & 15: return ((L + (rs & 15)) << 8) | ((rs >> 4) + 1)
        return {0: (L << 8) | 255, 0xF0: (L << 8) | 16}.get(rs, 0)

    DC = [c[0] for c in dc]; AC = [[conv(e) for e in c[0]] for c in ac]

    def bits(x, y): return (BIG >> (E - y)) & ((1 << (y - x)) - 1), y - x

    def blk(p, k):  # -> (cuối block, DC diff, cuối phần DC) | None
        e = DC[k][w[p >> 3] >> (8 - (p & 7)) & 65535]
        if not e: return
        s = e & 255; p += e >> 8; v = 0
        if s:
            v = (w[p >> 3] >> (8 - (p & 7)) & 65535) >> (16 - s)
            if v < 1 << (s - 1): v -= (1 << s) - 1
            p += s
        q, i, A = p, 1, AC[k]
        while i < 64:
            e = A[w[p >> 3] >> (8 - (p & 7)) & 65535]
            if not e: return
            p += e >> 8; c = e & 255
            if c == 255: break
            i += c
        if i <= 64 and p <= E: return p, v, q

    dead = set()  # ranh giới MCU đã biết dẫn tới fail: ứng viên hội tụ vào đây bị loại ngay

    def run(p, ph):
        vis = []
        r = _run(p, ph, vis)
        if not r: dead.update(vis)
        return r

    def _run(p, ph, vis):  # giải mã tới hết; đúng nếu kết thúc ở EOI + pad 1 -> (st, số MCU, tổng DC)
        for k in seq[ph:] if ph else ():  # MCU dở dang
            if not (r := blk(p, k)): return
            p = r[0]
        st, cnt, pred, S = p, 0, [0] * len(cs), [0] * len(cs)
        while True:
            r = E - p
            if r < 8:
                if r >= 0 and (w[p >> 3] >> (8 - (p & 7)) & 65535) >> (16 - r) == (1 << r) - 1: return st, cnt, S
                return
            if cnt == tot: return
            if cnt < 64:
                if p in dead: return
                vis.append(p)
            for k in seq:
                e = DC[k][w[p >> 3] >> (8 - (p & 7)) & 65535]
                if not e: return
                s = e & 255; p += e >> 8
                if s:
                    v = (w[p >> 3] >> (8 - (p & 7)) & 65535) >> (16 - s)
                    if v < 1 << (s - 1): v -= (1 << s) - 1
                    p += s; pred[k] += v
                S[k] += pred[k]
                i, A = 1, AC[k]
                while i < 64:
                    e = A[w[p >> 3] >> (8 - (p & 7)) & 65535]
                    if not e: return
                    p += e >> 8; c = e & 255
                    if c == 255: break
                    i += c
                if i > 64 or p > E: return
            cnt += 1

    res = next((r for p0 in range(8 * win) for ph in range(len(seq)) if (r := run(p0, ph))), None)
    if not res: raise ValueError('không tìm được điểm đồng bộ (DHT/kích thước trong header sai?)')
    st, cnt, S = res
    # bỏ >=skip MCU đầu (rác do Huffman tự đồng bộ) rồi căn tới đầu hàng MCU
    sk = skip + -(tot - cnt + skip) % mx
    if cnt <= sk: raise ValueError('còn lại quá ít MCU')
    rows = (tot - cnt + sk) // mx

    pred, first, p = [0] * len(cs), [], st
    for m in range(sk + 1):  # chỉ giải mã lại tới MCU đầu giữ lại
        for k in seq:
            q, v, x = blk(p, k)
            pred[k] += v
            if m == sk: first.append((k, pred[k], x, q, p))
            p = q

    def build(off):  # chỉ MCU đầu phải mã hoá lại (DC); phần đuôi chép nguyên bit
        pre, seen = (0, 0), set()
        for k, v, x, q, p in first:
            if k in seen: pre = cat(pre, bits(p, q)); continue
            seen.add(k); v += off[k]
            mxs = max(dc[k][1]); v = max(1 - (1 << mxs), min(v, (1 << mxs) - 1))
            s = abs(v).bit_length()
            pre = cat(cat(cat(pre, dc[k][1][s]), (v if v > 0 else v + (1 << s) - 1, s)), bits(x, q))
        return out(pack(*cat(pre, bits(first[-1][3], E))) + b'\xff\xd9', rows)

    # DC tuyệt đối đầu bị mất -> cả ảnh lệch một hằng số mỗi kênh
    if off is None: off = [-round(S[k] / (cnt * seq.count(k))) for k in range(len(cs))]  # mặc định: trung bình DC về 0
    else: off = list(off)
    if ref is not None:  # khớp trung bình YCbCr với ảnh tham chiếu (cùng cảnh) ở đúng vùng còn lại
        from PIL import Image
        im = ref if isinstance(ref, Image.Image) else Image.open(ref)
        R = np.asarray(im.convert('YCbCr').resize((W, H)))[rows * 8 * vm:].reshape(-1, 3).mean(0)
        for _ in range(3):  # lặp để bù clip
            M = np.asarray(Image.open(io.BytesIO(build(off))).convert('YCbCr')).reshape(-1, 3).mean(0)
            off = [o + round((R[k] - M[k]) * 8 / Q[cs[k][3]]) for k, o in enumerate(off)]
    i = first[0][4] >> 3; fo = se + i + d.count(b'\xff', 0, i)  # đổi về offset thật (cộng lại byte stuffing)
    return build(off), (f'MCU tốt đầu: #{tot - cnt + sk}/{tot} (hàng {rows}) tại offset {fo} bit {first[0][4] & 7}; '
                        f'sync gốc offset {se + st // 8}; cắt height {H} -> {H - rows * 8 * vm}; off DC {off}')

if __name__ == '__main__':
    a = sys.argv; kv = dict(x.split('=') for x in a[3:])
    try:
        r, info = fix(open(a[1], 'rb').read(), kv.get('ref'),
                      [int(x) for x in kv['off'].split(',')] if 'off' in kv else None,
                      int(kv.get('win', 256)), int(kv.get('skip', 8)))
    except ValueError as e: sys.exit(e)
    open(a[2], 'wb').write(r); print(info)
