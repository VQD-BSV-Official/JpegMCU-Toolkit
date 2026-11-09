#!/usr/bin/env python3
"""Sửa JPEG baseline (1 scan, không RST) có đoạn entropy data bị ghi đè/hỏng.
Giải hệ số DCT -> định vị vùng hỏng -> ghép lại phần sau (dịch theo tổng số MCU)
-> hiệu chỉnh DC predictor -> nội suy DC cho MCU mất -> mã hóa lại bằng bảng Huffman gốc.
Dùng: jpgfix.py in.jpg [out.jpg] [--g MCU_BẮT_ĐẦU_HỎNG] [--r MCU_ĐỒNG_BỘ_LẠI]"""
import sys, argparse, numpy as np


def parse(d):
    p, tabs, dri = 2, {}, 0
    while d[p] == 0xFF:
        m, ln = d[p + 1], int.from_bytes(d[p + 2:p + 4], 'big')
        s = d[p + 4:p + 2 + ln]
        if m == 0xC4:
            q = 0
            while q < len(s):
                cnt = s[q + 1:q + 17]; n = sum(cnt)
                tabs[s[q]] = (cnt, s[q + 17:q + 17 + n]); q += 17 + n
        elif m in (0xC0, 0xC1):
            assert s[0] == 8, 'chỉ hỗ trợ 8-bit'
            H, W = int.from_bytes(s[1:3], 'big'), int.from_bytes(s[3:5], 'big')
            fc = [(s[6 + 3 * i], s[7 + 3 * i] >> 4, s[7 + 3 * i] & 15) for i in range(s[5])]
        elif 0xC2 <= m <= 0xCF and m not in (0xC4, 0xC8, 0xCC):
            sys.exit('không hỗ trợ progressive/lossless/arithmetic')
        elif m == 0xDD: dri = int.from_bytes(s[:2], 'big')
        elif m == 0xDA:
            sc = {s[1 + 2 * i]: s[2 + 2 * i] for i in range(s[0])}
            assert len(sc) == len(fc), 'cần scan interleaved đủ thành phần'
            if dri: sys.exit('không hỗ trợ restart interval')
            return H, W, fc, sc, tabs, p + 2 + ln
        p += 2 + ln
    sys.exit('không thấy SOS')


def lut(t):
    cnt, syms = t; a = np.zeros((65536, 2), np.int32); code = k = 0
    for L in range(1, 17):
        for _ in range(cnt[L - 1]):
            lo = code << (16 - L); a[lo:lo + (1 << (16 - L))] = (L, syms[k]); code += 1; k += 1
        code <<= 1
    return a.tolist()


def decode(raw, blocks, tabs, sc, maxm):
    s = raw.replace(b'\xff\x00', b'\xff')  # FF lẻ trong vùng hỏng giữ nguyên như byte dữ liệu
    n = len(s) * 8
    b = np.unpackbits(np.frombuffer(s + b'\0\0\0', np.uint8))
    pk = np.zeros(n + 16 + 20000, np.uint32)
    for j in range(16): pk[:n] |= b[j:j + n].astype(np.uint32) << (15 - j)
    pk = pk.tolist()
    nb = len(blocks); S = np.zeros((maxm, nb, 64), np.int32)
    dl = {c: lut(tabs[sc[c] >> 4]) for c in sc}; al = {c: lut(tabs[0x10 | sc[c] & 15]) for c in sc}
    pred = {c: 0 for c in sc}; errs = []; pos = m = 0
    ext = lambda v, s: v if v >= 1 << (s - 1) else v - (1 << s) + 1
    while m < maxm and pos < n - 16:
        ok = True
        for bi, c in enumerate(blocks):
            blk = S[m, bi]; D, A = dl[c], al[c]
            ln, sy = D[pk[pos]]
            if not ln or sy > 11: ok = False; pos += 1; continue
            pos += ln
            if sy: pred[c] += ext(pk[pos] >> (16 - sy), sy); pos += sy
            blk[0] = pred[c]; k = 1
            while k < 64:
                ln, sy = A[pk[pos]]
                if not ln: ok = False; pos += 1; break
                pos += ln
                if not sy: break
                if sy == 0xF0: k += 16; continue
                k += sy >> 4; sz = sy & 15
                if k > 63: ok = False; break
                blk[k] = ext(pk[pos] >> (16 - sz), sz); pos += sz; k += 1
        if not ok: errs.append(m)
        m += 1
    return S[:m], errs


def encode(T, blocks, tabs, sc):
    def mk(t):
        cnt, syms = t; code = k = 0; e = {}
        for L in range(1, 17):
            for _ in range(cnt[L - 1]): e[syms[k]] = (code, L); code += 1; k += 1
            code <<= 1
        return e
    ed = {c: mk(tabs[sc[c] >> 4]) for c in sc}; ea = {c: mk(tabs[0x10 | sc[c] & 15]) for c in sc}
    buf = bytearray(); st = [0, 0]  # acc, nbits

    def put(v, n):
        a = (st[0] << n) | (v & ((1 << n) - 1)); nb = st[1] + n
        while nb >= 8:
            nb -= 8; x = (a >> nb) & 255; buf.append(x)
            if x == 255: buf.append(0)
        st[0], st[1] = a & ((1 << nb) - 1), nb
    pred = {c: 0 for c in sc}
    for m in range(len(T)):
        for bi, c in enumerate(blocks):
            b = T[m, bi]; df = max(-2047, min(2047, int(b[0]) - pred[c])); pred[c] += df
            s = abs(df).bit_length(); put(*ed[c][s])
            if s: put(df if df > 0 else df + (1 << s) - 1, s)
            nz = np.flatnonzero(b[1:]) + 1; last = nz[-1] if len(nz) else 0; run = 0; A = ea[c]
            for k in range(1, last + 1):
                v = int(b[k])
                if not v: run += 1; continue
                while run > 15: put(*A[0xF0]); run -= 16
                s = abs(v).bit_length(); put(*A[run << 4 | s]); put(v if v > 0 else v + (1 << s) - 1, s); run = 0
            if last < 63: put(*A[0])
    if st[1]: put((1 << (8 - st[1])) - 1, 8 - st[1])
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('i'); ap.add_argument('o', nargs='?')
    ap.add_argument('--g', type=int); ap.add_argument('--r', type=int)
    a = ap.parse_args(); o = a.o or a.i.rsplit('.', 1)[0] + '_fixed.jpg'
    d = open(a.i, 'rb').read()
    H, W, fc, sc, tabs, ss = parse(d)
    hm, vm = max(f[1] for f in fc), max(f[2] for f in fc)
    if len(fc) == 1: hm = vm = 1; fc = [(fc[0][0], 1, 1)]
    mw, mh = -(-W // (8 * hm)), -(-H // (8 * vm)); N = mw * mh
    blocks = [c for c, h, v in fc for _ in range(h * v)]
    end = d.rfind(b'\xff\xd9'); end = len(d) if end < ss else end
    S, errs = decode(d[ss:end], blocks, tabs, sc, N + 3000)
    M = len(S); K = M - N
    print(f'{W}x{H} MCU {mw}x{mh}={N}, giải được {M} (lệch {K:+d}), MCU lỗi: {len(errs)}')
    if not errs and not K and a.g is None:
        print('không phát hiện hỏng'); open(o, 'wb').write(d[:ss] + encode(S, blocks, tabs, sc) + b'\xff\xd9'); return
    first = {c: blocks.index(c) for c in sc}
    G = a.g if a.g is not None else max(errs[0] - 1, 0)
    if a.r is not None: R = a.r
    else:
        R = errs[-1] + 1; bad = []
        for c in first:  # bước nhảy DC bất thường theo từng khối, trong cửa sổ sau lỗi cuối
            nb_ = blocks.count(c); sq = S[:, [i for i, x in enumerate(blocks) if x == c], 0].reshape(-1)
            st = np.abs(np.diff(sq, prepend=sq[0])); th = max(6 * np.percentile(st[1:max(G * nb_, 2)], 95), 64)
            bad += [j // nb_ for j in range(errs[-1] * nb_, min((errs[-1] + 24) * nb_, len(sq))) if st[j] > th]
        R = max(bad + [R - 1]) + 1
    t0 = R - K
    assert 0 <= G <= t0 <= N, f'G={G} R={R} t0={t0} không hợp lệ, thử --g/--r'
    print(f'hỏng từ MCU {G}, đồng bộ lại tại {R} -> vị trí thật {t0}; mất {t0 - G} MCU')
    T = np.zeros((N, len(blocks), 64), np.int32); valid = np.zeros(N, bool)
    T[:G] = S[:G]; valid[:G] = True
    n = min(M - R, N - t0); T[t0:t0 + n] = S[R:R + n]; valid[t0:t0 + n] = True
    V = valid.reshape(mh, mw)
    # offset DC từng thành phần: so MCU đầu tiên sau vùng mất với MCU hợp lệ gần nhất phía trên cùng cột
    for c, fb in first.items():
        ib = [i for i, x in enumerate(blocks) if x == c]; mean = lambda t: T[t, ib, 0].mean()
        ds = []
        for col in range(mw):
            idx = np.arange(mh) * mw + col; r = np.arange(mh)
            pre, post = r[V[:, col] & (idx < G)], r[V[:, col] & (idx >= t0)]
            if len(pre) and len(post): ds.append((post[0] - pre[-1], mean(post[0] * mw + col) - mean(pre[-1] * mw + col)))
        if not ds: print(f'cảnh báo: không có mốc DC cho thành phần {c}'); continue
        g0 = min(x[0] for x in ds); off = int(round(np.median([x[1] for x in ds if x[0] == g0])))
        T[t0:t0 + n][:, ib, 0] -= off; print(f'offset DC thành phần {c}: {off:+d}')
    # lấp MCU mất: nội suy DC dọc, AC = 0
    for col in range(mw):
        ok = np.flatnonzero(V[:, col])
        for r in np.flatnonzero(~V[:, col]):
            ra, rb = ok[ok < r], ok[ok > r]
            A = T[ra[-1] * mw + col] if len(ra) else None; B = T[rb[0] * mw + col] if len(rb) else None
            w = (r - ra[-1]) / (rb[0] - ra[-1]) if A is not None and B is not None else 0
            A = B if A is None else A; B = A if B is None else B
            for c in sc:
                ib = [i for i, x in enumerate(blocks) if x == c]
                T[r * mw + col, ib, 0] = int(round((1 - w) * A[ib, 0].mean() + w * B[ib, 0].mean()))
    open(o, 'wb').write(d[:ss] + encode(T, blocks, tabs, sc) + b'\xff\xd9'); print('->', o)


if __name__ == '__main__':
    main()
