#!/usr/bin/env python3
"""
track_jultagi.py — 실사 줄타기 영상(jultagi_6s_stab.webm)에서 광대의 휘청(sway) 운동을 추출한다.

사용:  python tools/track_jultagi.py [check_dir]
출력
  static/media/jultagi_trace.json   프레임별 foot/torso/head 픽셀좌표 + tilt + 루프 구간 + 줄·기둥 기하(프레임별)
  <check_dir>/track_check.png       3.0~6.0 s 8프레임 검출 확인 스트립
  <check_dir>/track_plot.png        foot_x / torso_x·head_x / tilt_deg / upper·fold vs t

의존성: numpy, scipy(ndimage, optimize), ffmpeg 바이너리(imageio_ffmpeg 동봉). PIL/matplotlib/cv2 불필요.

방법(요약)
  * ffmpeg rawvideo 로 (a) 광대 주변 500x500 crop, (b) 줄 높이 대역(1080x300) 두 스트림을 디코드.
  * torso = 조끼(짙은 남색: B>G+5, B>R+15, R<90, G<100) 최대 연결성분 무게중심(직전 검출 ±90 px 창).
  * foot  = 조끼 아래 띠에서 청백/회백(바지·버선) 성분마다 최하단점을 후보로 잡고,
            그 프레임의 줄 적합선에 가장 가까운 후보(= 줄에 닿은 발)를 택한다. 걷는 동안 들린 뒷발 배제.
  * head  = 갓+얼굴(주황/황갈/살색: R>120, R>G+8, G>B+4, R−B>25) 성분 중 직전 머리에 가장 가까운 것의 무게중심.
            부채·소매가 머리 위로 올라오므로 "가장 위 흰 픽셀"은 쓰지 않는다.
  * tilt_deg = atan2(torso_x − foot_x, foot_y − torso_y) [deg], 화면상 torso 가 발 오른쪽이면 +.
    upper_deg = torso→head 벡터의 연직 기준 각, fold_deg = upper − tilt(허리 접힘, 머리가 엉덩이보다 왼쪽이면 −),
    lean_deg = foot→head.
  * 줄: (R+G)/2 ridge 필터 열별 argmax → 발 좌/우 두 직선 RANSAC → 꼭짓점·발 아래 줄 y·기둥 교차점에서의 줄 y.
  * 기둥 교차점(줄 끝): 대나무(밝은 황갈) 마스크에 RANSAC 직선 여러 개 → 가파른 두 직선(각도차>20°)의 교점,
    프레임마다 추적. 안정화 영상이지만 잔여 드리프트(좌 기둥 ≈ (−24,+33) px/6 s)가 있어 프레임별로 준다.
"""
import json, os, struct, subprocess, sys, zlib
import numpy as np
from scipy import ndimage, optimize

HERE = os.path.dirname(os.path.abspath(__file__))
V22 = os.path.dirname(HERE)
FF = '/usr/local/lib/python3.11/dist-packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
VID = os.path.join(os.path.dirname(V22), 'presentation', 'media', 'jultagi_6s_stab.webm')
OUT_JSON = os.path.join(V22, 'static', 'media', 'jultagi_trace.json')
CHECK_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.join(V22, 'video_out', 'vf')
FPS = 30.0
W, H = 1080, 1920
# 광대 crop (전체 프레임 기준 오프셋). 광대는 x≈600~760, y≈760(부채)~1070(발) 에 있다.
CX0, CY0, CW, CH = 450, 650, 500, 500
# 줄·기둥 대역
RY0, RH = 900, 300
# 기둥 교차점 초기 추정(프레임 0, 전체 프레임 px) — 이후 프레임마다 재검출
POST_L0, POST_R0 = (205.0, 1072.0), (885.0, 980.0)
RNG = np.random.default_rng(0)


# ─────────────────────────────── I/O ───────────────────────────────
def decode(vf, w, h):
    cmd = [FF, '-hide_banner', '-loglevel', 'error', '-i', VID, '-vf', vf,
           '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-']
    out = subprocess.run(cmd, capture_output=True, check=True).stdout
    n = len(out) // (w * h * 3)
    return np.frombuffer(out, np.uint8)[:n * w * h * 3].reshape(n, h, w, 3)


def write_png(path, arr):
    arr = np.ascontiguousarray(arr.astype(np.uint8))
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, 2)
    h, w = arr.shape[:2]
    raw = b''.join(b'\x00' + arr[y].tobytes() for y in range(h))

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(png)


# ─────────────────────────── 색 분할 마스크 ───────────────────────────
def _rgb(img):
    return [img[..., i].astype(np.int16) for i in range(3)]


def mask_vest(img):
    r, g, b = _rgb(img)
    return (b > g + 5) & (b > r + 15) & (r < 90) & (g < 100) & (b < 170)


def mask_white(img):
    """청백/회백(바지·버선·소매). 노란빛 줄(G>B), 하늘(채도 큼), 나무 배제."""
    r, g, b = _rgb(img)
    mn = np.minimum(np.minimum(r, g), b)
    mx = np.maximum(np.maximum(r, g), b)
    return (mn > 110) & (mx - mn < 45) & (b >= g - 5)


def mask_head(img):
    """갓(주황·황갈) + 얼굴(살색). 그늘진 갓 (150,140,125) 까지 포함하도록 느슨하게."""
    r, g, b = _rgb(img)
    return (r > 120) & (r > g + 8) & (g > b + 4) & (r - b > 25)


def mask_pole(img):
    """대나무 기둥·줄(밝은 황갈). 하늘(B>G)·나무(어두움) 배제."""
    r, g, b = _rgb(img)
    return ((r + g) // 2 > 140) & (b < g - 8) & (r > 120)


def components(m, min_area=1):
    """연결성분 목록 [(area, cx, cy, label)], lab 배열."""
    lab, n = ndimage.label(m)
    out = []
    if n:
        idx = np.arange(1, n + 1)
        areas = ndimage.sum(np.ones_like(m, dtype=np.int32), lab, index=idx)
        cms = ndimage.center_of_mass(m, lab, index=idx)
        for a, (cy, cx), k in zip(areas, cms, idx):
            if a >= min_area:
                out.append((int(a), float(cx), float(cy), int(k)))
    return out, lab


def pick_component(comps, near, max_dist=None):
    """near=(x,y) 에 가장 가까운(면적 보너스 0.02·area) 성분."""
    best = None
    for a, cx, cy, k in comps:
        d = np.hypot(cx - near[0], cy - near[1])
        if max_dist is not None and d > max_dist:
            continue
        s = d - 0.02 * a
        if best is None or s < best[0]:
            best = (s, a, cx, cy, k)
    return best


# ───────────────────────────── 직선 적합 ─────────────────────────────
def fit_line_ransac(xs, ys, iters=300, thr=2.5):
    """y = a x + b (x 함수형). 반환 (a, b, inliers, samples) 또는 None."""
    xs = xs.astype(float); ys = ys.astype(float)
    if len(xs) < 10:
        return None
    best, best_in = None, 0
    for _ in range(iters):
        i, j = RNG.choice(len(xs), 2, replace=False)
        if xs[i] == xs[j]:
            continue
        a = (ys[j] - ys[i]) / (xs[j] - xs[i]); b = ys[i] - a * xs[i]
        inl = np.abs(ys - (a * xs + b)) < thr
        if inl.sum() > best_in:
            best_in, best = inl.sum(), inl
    a, b = np.polyfit(xs[best], ys[best], 1)
    return float(a), float(b), int(best_in), int(len(xs))


def ransac_lines_general(ys, xs, nl=4, thr=2.0, rm_thr=7.0, iters=250, min_in=40):
    """방향 제한 없는 직선(점 c + s·u) 을 순차 RANSAC 으로 nl 개까지. 같은 굵은 기둥을 두 번 잡지 않도록
    검출 후 rm_thr 폭으로 내점을 제거한다. 반환 [(c, u, n_inliers)]."""
    P = np.stack([xs, ys], 1).astype(float); lines = []
    for _ in range(nl):
        if len(P) < min_in:
            break
        best = None
        for _ in range(iters):
            i, j = RNG.choice(len(P), 2, replace=False); d = P[j] - P[i]; nrm = np.hypot(*d)
            if nrm < 8:
                continue
            nv = np.array([-d[1], d[0]]) / nrm
            dist = np.abs((P - P[i]) @ nv)
            cnt = int((dist < thr).sum())
            if best is None or cnt > best[0]:
                best = (cnt, P[i], nv)
        if best is None or best[0] < min_in:
            break
        cnt, p0, nv = best
        dist = np.abs((P - p0) @ nv)
        # 굵은 기둥의 중심선: 2 px 내점이 아니라 rm_thr 폭 안의 모든 픽셀로 PCA 재적합(한쪽 가장자리 편향 제거)
        Q = P[dist < rm_thr]; c = Q.mean(0); u = np.linalg.svd(Q - c)[2][0]
        d2 = np.abs((P - c) @ np.array([-u[1], u[0]]))
        Q = P[d2 < rm_thr]; c = Q.mean(0); u = np.linalg.svd(Q - c)[2][0]
        lines.append((c, u, int(cnt)))
        P = P[d2 >= rm_thr]
    return lines


def find_crossing(band_img, guess, half=100):
    """기둥 교차점: 대나무 마스크 → 직선들 → 수평에서 ≥29° 기운 두 직선(각도차 >20°) 의 교점. 전체 프레임 px."""
    gx, gy = guess[0], guess[1] - RY0
    x0 = int(np.clip(gx - half, 0, W - 2 * half)); y0 = int(np.clip(gy - half, 0, RH - 2 * half))
    patch = band_img[y0:y0 + 2 * half, x0:x0 + 2 * half]
    ys, xs = np.nonzero(mask_pole(patch))
    if len(xs) < 80:
        return None
    lines = ransac_lines_general(ys, xs)
    steep = [(c, u, n) for c, u, n in lines if abs(u[1]) > 0.55 * abs(u[0])]
    if len(steep) < 2:
        return None
    ang = lambda u: np.degrees(np.arctan2(u[1], u[0])) % 180.0
    c1, u1, n1 = steep[0]
    second = None
    for c2, u2, n2 in steep[1:]:
        d = abs(ang(u1) - ang(u2)); d = min(d, 180 - d)
        if d > 20:
            second = (c2, u2, n2); break
    if second is None:
        return None
    c2, u2, n2 = second
    A = np.array([u1, -u2]).T
    try:
        s, t = np.linalg.solve(A, c2 - c1)
    except np.linalg.LinAlgError:
        return None
    p = c1 + s * u1
    return (float(p[0] + x0), float(p[1] + y0 + RY0), n1, n2)


# ──────────────────────────────── 줄 검출 ────────────────────────────────
def rope_ridge_rows(band_img, x_lo, x_hi, y_lo, y_hi, gap=6):
    """열별 ridge(밝은 얇은 수평선) argmax → (xs, ys) band 좌표."""
    L = band_img[..., :2].astype(np.float32).mean(-1)  # (R+G)/2 : 노란 줄에 민감, 파란 하늘엔 둔감
    ridge = np.full_like(L, -1e9)
    ridge[gap:-gap] = L[gap:-gap] - 0.5 * (L[:-2 * gap] + L[2 * gap:])
    y_lo = int(max(gap, y_lo)); y_hi = int(min(RH - gap, y_hi))
    sub = ridge[y_lo:y_hi, x_lo:x_hi]
    ys = sub.argmax(0) + y_lo
    strength = sub.max(0)
    xs = np.arange(x_lo, x_hi)
    ok = strength > 35
    return xs[ok], ys[ok]


def rope_geometry(band_img, foot_full, x_left, x_right):
    """foot 좌/우 두 직선(전체 프레임 좌표 y = a x + b). x_left/x_right: 기둥 교차점 x."""
    fx, fy = foot_full
    yl, yh = fy - RY0 - 70, fy - RY0 + 40
    res = {}
    for side, (xa, xb, pad) in dict(left=(int(x_left) + 15, int(fx) - 45, 60), right=(int(fx) + 45, int(x_right) - 15, 40)).items():
        if xb - xa < 30:
            res[side] = None; continue
        xs, ys = rope_ridge_rows(band_img, xa, xb, yl - pad, yh + 40)
        fit = fit_line_ransac(xs, ys)
        if fit is None:
            res[side] = None; continue
        a, b, nin, ntot = fit
        res[side] = dict(a=a, b=b + RY0, inliers=nin, samples=ntot)
    if res.get('left') and res.get('right'):
        al, bl = res['left']['a'], res['left']['b']; ar, br = res['right']['a'], res['right']['b']
        xv = (br - bl) / (al - ar) if abs(al - ar) > 1e-6 else fx
        res['vertex'] = (float(xv), float(al * xv + bl))
        res['y_at'] = lambda x: float(min(al * x + bl, ar * x + br))
        res['end_left'] = (float(x_left), float(al * x_left + bl))
        res['end_right'] = (float(x_right), float(ar * x_right + br))
    return res


# ───────────────────────────── 프레임 검출 ─────────────────────────────
def detect(img, prev, rope_y=None):
    """img: crop 프레임. prev: 직전 상태 dict(torso, head) 또는 None. rope_y: 전체프레임 x→줄 y 함수(없으면 최하단 후보).
    반환 dict(crop 좌표계) 또는 None(조끼 없음)."""
    hgt, wid = img.shape[:2]
    win = np.zeros((hgt, wid), bool)
    if prev is None:
        win[:] = True
    else:
        x0, y0 = prev['torso']
        win[max(0, int(y0) - 90):int(y0) + 90, max(0, int(x0) - 90):int(x0) + 90] = True
    comps, lab = components(mask_vest(img) & win, 40)
    if not comps:
        return None
    if prev is None:
        a, cx, cy, k = max(comps)
    else:
        pk = pick_component(comps, prev['torso'])
        _, a, cx, cy, k = pk
    if a < 150:
        return None
    comp = lab == k
    ys, xs = np.nonzero(comp)
    tx, ty = xs.mean(), ys.mean()
    v_top, v_bot = int(ys.min()), int(ys.max())
    out = dict(torso=(tx, ty), vest_area=int(a), vest_bbox=(int(xs.min()), v_top, int(xs.max()), v_bot))

    # ── foot: 조끼 아래 띠의 흰 성분별 최하단점 후보 → 줄에 가장 가까운 것
    band = np.zeros((hgt, wid), bool)
    band[max(0, v_bot - 12): min(hgt, v_bot + 180), max(0, int(tx) - 90): int(tx) + 90] = True
    wcomps, wlab = components(mask_white(img) & band, 60)
    cands = []
    for wa, wcx, wcy, wk in wcomps:
        yy, xx = np.nonzero(wlab == wk)
        fy = int(yy.max()); fx = float(xx[yy >= fy - 5].mean())
        d = abs(fy + CY0 - rope_y(fx + CX0)) if rope_y else -fy
        cands.append((d, fx, fy, wa))
    if not cands:
        out['foot'] = None
    else:
        # 줄 거리 우선, 직전 발 x 와의 거리는 약한 가중(뒷발·소매 배제)
        px = prev['foot'][0] if (prev and prev.get('foot')) else tx
        d, fx, fy, wa = min(cands, key=lambda c: c[0] + 0.05 * abs(c[1] - px))
        out['foot'] = (fx, float(fy)); out['legs_area'] = int(wa); out['foot_rope_dist'] = float(d)
        out['foot_ncand'] = len(cands)

    # ── head: 조끼 위 창의 갓+얼굴 성분, 직전 머리 위치 연속성
    hb = np.zeros((hgt, wid), bool)
    hb[max(0, v_top - 130): v_top + 10, max(0, int(tx) - 70): int(tx) + 70] = True
    hcomps, hlab = components(mask_head(img) & hb, 20)
    near = prev['head'] if (prev and prev.get('head')) else (tx, v_top - 40)
    pk = pick_component(hcomps, near, max_dist=45 if (prev and prev.get('head')) else 70)
    if pk is None:
        out['head'] = None
    else:
        _, ha, hx, hy, hk = pk
        out['head'] = (hx, hy); out['head_area'] = int(ha)
    return out


# ───────────────────────────── 그림 유틸(numpy) ─────────────────────────────
def draw_disc(img, x, y, r, col):
    h, w = img.shape[:2]
    if not (np.isfinite(x) and np.isfinite(y)):
        return
    ya, yb, xa, xb = max(0, int(y) - r), min(h, int(y) + r + 1), max(0, int(x) - r), min(w, int(x) + r + 1)
    if yb <= ya or xb <= xa:
        return
    yy, xx = np.ogrid[ya:yb, xa:xb]
    m = (yy - y) ** 2 + (xx - x) ** 2 <= r * r
    img[ya:yb, xa:xb][m] = col


def draw_line(img, x0, y0, x1, y1, col, thick=1):
    if not all(np.isfinite(v) for v in (x0, y0, x1, y1)):
        return
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    for t in np.linspace(0, 1, max(2, n)):
        x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        if thick <= 1:
            if 0 <= int(y) < img.shape[0] and 0 <= int(x) < img.shape[1]:
                img[int(y), int(x)] = col
        else:
            draw_disc(img, x, y, thick // 2, col)


FONT = {  # 5x7 비트맵, 라벨용 최소 글꼴
    '0': ['01110', '10001', '10011', '10101', '11001', '10001', '01110'],
    '1': ['00100', '01100', '00100', '00100', '00100', '00100', '01110'],
    '2': ['01110', '10001', '00001', '00010', '00100', '01000', '11111'],
    '3': ['11111', '00010', '00100', '00010', '00001', '10001', '01110'],
    '4': ['00010', '00110', '01010', '10010', '11111', '00010', '00010'],
    '5': ['11111', '10000', '11110', '00001', '00001', '10001', '01110'],
    '6': ['00110', '01000', '10000', '11110', '10001', '10001', '01110'],
    '7': ['11111', '00001', '00010', '00100', '01000', '01000', '01000'],
    '8': ['01110', '10001', '10001', '01110', '10001', '10001', '01110'],
    '9': ['01110', '10001', '10001', '01111', '00001', '00010', '01100'],
    '.': ['00000', '00000', '00000', '00000', '00000', '01100', '01100'],
    '-': ['00000', '00000', '00000', '11111', '00000', '00000', '00000'],
    '+': ['00000', '00100', '00100', '11111', '00100', '00100', '00000'],
    '_': ['00000', '00000', '00000', '00000', '00000', '00000', '11111'],
    ' ': ['00000'] * 7,
    '=': ['00000', '00000', '11111', '00000', '11111', '00000', '00000'],
    '/': ['00001', '00010', '00010', '00100', '01000', '01000', '10000'],
    '[': ['01110', '01000', '01000', '01000', '01000', '01000', '01110'],
    ']': ['01110', '00010', '00010', '00010', '00010', '00010', '01110'],
    ',': ['00000', '00000', '00000', '00000', '01100', '00100', '01000'],
    'a': ['00000', '00000', '01110', '00001', '01111', '10001', '01111'],
    'b': ['10000', '10000', '10110', '11001', '10001', '10001', '11110'],
    'd': ['00001', '00001', '01101', '10011', '10001', '10001', '01111'],
    'e': ['00000', '00000', '01110', '10001', '11111', '10000', '01110'],
    'f': ['00110', '01001', '01000', '11100', '01000', '01000', '01000'],
    'g': ['00000', '00000', '01111', '10001', '01111', '00001', '01110'],
    'h': ['10000', '10000', '10110', '11001', '10001', '10001', '10001'],
    'i': ['00100', '00000', '01100', '00100', '00100', '00100', '01110'],
    'l': ['01100', '00100', '00100', '00100', '00100', '00100', '01110'],
    'm': ['00000', '00000', '11010', '10101', '10101', '10001', '10001'],
    'n': ['00000', '00000', '10110', '11001', '10001', '10001', '10001'],
    'o': ['00000', '00000', '01110', '10001', '10001', '10001', '01110'],
    'p': ['00000', '00000', '11110', '10001', '11110', '10000', '10000'],
    'r': ['00000', '00000', '10110', '11001', '10000', '10000', '10000'],
    's': ['00000', '00000', '01111', '10000', '01110', '00001', '11110'],
    't': ['01000', '01000', '11100', '01000', '01000', '01001', '00110'],
    'u': ['00000', '00000', '10001', '10001', '10001', '10011', '01101'],
    'x': ['00000', '00000', '10001', '01010', '00100', '01010', '10001'],
    'y': ['00000', '00000', '10001', '10001', '01111', '00001', '01110'],
}


def draw_text(img, x, y, s, col, scale=2):
    for ch in s:
        glyph = FONT.get(ch, FONT[' '])
        for r, row in enumerate(glyph):
            for c, bit in enumerate(row):
                if bit == '1':
                    img[y + r * scale:y + (r + 1) * scale, x + c * scale:x + (c + 1) * scale] = col
        x += 6 * scale


# ─────────────────────────────── 신호 처리 ───────────────────────────────
def interp_nan(a):
    a = np.asarray(a, float).copy()
    bad = ~np.isfinite(a)
    if bad.any() and (~bad).sum() >= 2:
        idx = np.arange(len(a))
        a[bad] = np.interp(idx[bad], idx[~bad], a[~bad])
    return a


def medfilt(a, k):
    return ndimage.median_filter(np.asarray(a, float), size=k, mode='nearest')


def sinus_fit(t, y, pmin=0.5, pmax=4.0):
    """y ≈ c0 + c1 t + A sin(2π t/P + φ), P∈[pmin,pmax] 경계 비선형 적합(다중 초기값)."""
    t = np.asarray(t, float); y = np.asarray(y, float)
    lin = np.polyfit(t, y, 1); d = y - np.polyval(lin, t)
    ac = np.correlate(d, d, 'full')[len(d) - 1:]; ac = ac / (ac[0] + 1e-12)
    lag = None
    for k in range(3, len(ac) - 1):
        if ac[k] > ac[k - 1] and ac[k] >= ac[k + 1] and ac[k] > 0:
            lag = k; break
    P_ac = (lag / FPS) if lag else float('nan')
    Fm = np.abs(np.fft.rfft(d * np.hanning(len(d)), n=8192)); fr = np.fft.rfftfreq(8192, 1 / FPS)
    k = int(np.argmax(Fm[1:])) + 1; P_fft = 1 / fr[k]

    def model(p, tt):
        return p[0] + p[1] * tt + p[2] * np.sin(2 * np.pi * tt / p[3] + p[4])
    best = None
    starts = [p for p in (P_ac, P_fft, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0) if np.isfinite(p) and pmin <= p <= pmax]
    for P0 in starts:
        for ph in (0, np.pi / 2, np.pi, 3 * np.pi / 2):
            p0 = [lin[1], lin[0], max(d.std() * 1.4, 1e-3), P0, ph]
            try:
                r = optimize.least_squares(lambda p: model(p, t) - y, p0,
                                           bounds=([-np.inf, -np.inf, 0, pmin, -np.inf], [np.inf, np.inf, np.inf, pmax, np.inf]))
            except Exception:
                continue
            if best is None or r.cost < best.cost:
                best = r
    p = best.x
    return dict(period=float(p[3]), amp=float(p[2]), rms_resid=float(np.sqrt(2 * best.cost / len(t))),
                ac_period=float(P_ac), fft_period=float(P_fft), p2p_half=float((d.max() - d.min()) / 2),
                trend_per_s=float(p[1]), model=lambda tt: model(p, tt))


# ──────────────────────────────── main ────────────────────────────────
def main():
    print('decoding crop...', flush=True)
    crop = decode(f'crop={CW}:{CH}:{CX0}:{CY0}', CW, CH)
    print('decoding rope band...', flush=True)
    band = decode(f'crop={W}:{RH}:0:{RY0}', W, RH)
    n = crop.shape[0]
    t = np.arange(n) / FPS
    print('frames', n, flush=True)

    keys = ['foot_x', 'foot_y', 'torso_x', 'torso_y', 'head_x', 'head_y']
    raw = {k: np.full(n, np.nan) for k in keys}
    vest_area = np.zeros(n, int); legs_area = np.zeros(n, int); head_area = np.zeros(n, int)
    foot_rope_dist = np.full(n, np.nan); foot_ncand = np.zeros(n, int)
    post_l = np.full((n, 2), np.nan); post_r = np.full((n, 2), np.nan)
    rope_l = np.full((n, 2), np.nan); rope_r = np.full((n, 2), np.nan)
    rope_vertex = np.full((n, 2), np.nan); rope_yfoot = np.full(n, np.nan)
    rope_end_l = np.full((n, 2), np.nan); rope_end_r = np.full((n, 2), np.nan)
    rope_inl = np.zeros((n, 4), int)
    fail = dict(torso=[], foot=[], head=[], post_left=[], post_right=[], rope=[])
    prev = None
    pl, pr = POST_L0, POST_R0
    foot_guess = None
    for i in range(n):
        # 기둥 교차점(줄 끝) 추적
        for side, guess, arr in (('left', pl, post_l), ('right', pr, post_r)):
            c = find_crossing(band[i], guess)
            gate = 45 if not np.isfinite(arr[:i, 0]).any() else 30   # 첫 검출 전엔 시드 오차 허용
            if c is None or np.hypot(c[0] - guess[0], c[1] - guess[1]) > gate:
                fail['post_' + side].append(i)
            else:
                arr[i] = c[:2]
                if side == 'left': pl = c[:2]
                else: pr = c[:2]
        # 줄(직전 발 또는 조끼 기반 추정으로 분할)
        rope = None
        if foot_guess is None:
            d0 = detect(crop[i], prev, None)
            if d0 is not None:
                bx = d0['torso'][0] + CX0
                by = (d0['foot'][1] + CY0) if d0.get('foot') else d0['vest_bbox'][3] + CY0 + 120
                foot_guess = (bx, by)
        if foot_guess is not None:
            rope = rope_geometry(band[i], foot_guess, pl[0], pr[0])
            if not rope.get('vertex'):
                rope = None
        d = detect(crop[i], prev, rope['y_at'] if rope else None)
        if d is None:
            fail['torso'].append(i); fail['foot'].append(i); fail['head'].append(i)
            continue
        tx, ty = d['torso']
        raw['torso_x'][i] = tx + CX0; raw['torso_y'][i] = ty + CY0
        vest_area[i] = d['vest_area']
        if d.get('foot') is None:
            fail['foot'].append(i)
        else:
            fx, fy = d['foot'][0] + CX0, d['foot'][1] + CY0
            raw['foot_x'][i] = fx; raw['foot_y'][i] = fy
            legs_area[i] = d.get('legs_area', 0); foot_rope_dist[i] = d.get('foot_rope_dist', np.nan); foot_ncand[i] = d.get('foot_ncand', 0)
            if foot_guess is None or abs(fx - foot_guess[0]) > 20 or abs(fy - foot_guess[1]) > 20:
                r2 = rope_geometry(band[i], (fx, fy), pl[0], pr[0])
                if r2.get('vertex'):
                    rope = r2
            foot_guess = (fx, fy)
        if d.get('head') is None:
            fail['head'].append(i)
        else:
            raw['head_x'][i] = d['head'][0] + CX0; raw['head_y'][i] = d['head'][1] + CY0; head_area[i] = d.get('head_area', 0)
        if rope:
            rope_l[i] = (rope['left']['a'], rope['left']['b']); rope_r[i] = (rope['right']['a'], rope['right']['b'])
            rope_vertex[i] = rope['vertex']; rope_end_l[i] = rope['end_left']; rope_end_r[i] = rope['end_right']
            rope_inl[i] = (rope['left']['inliers'], rope['left']['samples'], rope['right']['inliers'], rope['right']['samples'])
            if np.isfinite(raw['foot_x'][i]):
                rope_yfoot[i] = rope['y_at'](raw['foot_x'][i])
        else:
            fail['rope'].append(i)
        prev = dict(torso=(tx, ty), foot=d.get('foot'), head=d.get('head'))
        if i % 30 == 0:
            print('  frame %d' % i, flush=True)

    # 실패 프레임 보간
    filled = {k: interp_nan(v) for k, v in raw.items()}
    post_l = np.stack([interp_nan(post_l[:, 0]), interp_nan(post_l[:, 1])], 1)
    post_r = np.stack([interp_nan(post_r[:, 0]), interp_nan(post_r[:, 1])], 1)
    post_l_s = np.stack([medfilt(post_l[:, 0], 5), medfilt(post_l[:, 1], 5)], 1)
    post_r_s = np.stack([medfilt(post_r[:, 0], 5), medfilt(post_r[:, 1], 5)], 1)
    for arr in (rope_l, rope_r, rope_vertex, rope_end_l, rope_end_r):
        arr[:, 0] = interp_nan(arr[:, 0]); arr[:, 1] = interp_nan(arr[:, 1])
    rope_yfoot = interp_nan(rope_yfoot)

    # 각도(연직 기준, 화면 오른쪽 +)
    def ang(x_from, y_from, x_to, y_to):
        return np.degrees(np.arctan2(x_to - x_from, y_from - y_to))
    tilt_raw = ang(filled['foot_x'], filled['foot_y'], filled['torso_x'], filled['torso_y'])
    upper_raw = ang(filled['torso_x'], filled['torso_y'], filled['head_x'], filled['head_y'])
    lean_raw = ang(filled['foot_x'], filled['foot_y'], filled['head_x'], filled['head_y'])
    # 가벼운 평활: 위치 3프레임 중앙값, 각도 5프레임 중앙값
    sm = {k: medfilt(v, 3) for k, v in filled.items()}
    tilt = medfilt(tilt_raw, 5); upper = medfilt(upper_raw, 5); lean = medfilt(lean_raw, 5)
    fold = upper - tilt

    # ── 서 있는 구간 시작: 3.0 s 이후, 이후로 발 x 가 프레임당 4 px 미만으로만 변하고(발 바꿈 없음) 최종 중앙값 ±20 px 안
    fin = np.median(sm['foot_x'][-30:]); tfin = np.median(sm['torso_x'][-30:])
    step = np.abs(np.diff(sm['foot_x']))
    stand = None
    for i in range(int(3.0 * FPS), n - 15):
        if step[i:].max() < 4.0 and abs(sm['foot_x'][i] - fin) < 15 and abs(sm['torso_x'][i] - tfin) < 15:
            stand = i; break
    if stand is None:
        stand = int(3.6 * FPS)
    t_stand = stand / FPS
    ssel = np.arange(n) >= stand

    # ── 휘청 분석 (서 있는 구간)
    ana = {}
    fits = {}
    for name, sig in [('foot_x', sm['foot_x']), ('torso_x', sm['torso_x']), ('head_x', sm['head_x']),
                      ('tilt_deg', tilt), ('upper_deg', upper), ('fold_deg', fold), ('lean_deg', lean)]:
        f = sinus_fit(t[ssel], sig[ssel]); fits[name] = f
        ana[name] = dict(period_s=round(f['period'], 3), amp=round(f['amp'], 2), p2p_half=round(f['p2p_half'], 2),
                         rms_resid=round(f['rms_resid'], 2), ac_period_s=None if not np.isfinite(f['ac_period']) else round(f['ac_period'], 3),
                         fft_period_s=round(f['fft_period'], 3), trend_per_s=round(f['trend_per_s'], 2),
                         mean=round(float(sig[ssel].mean()), 2), min=round(float(sig[ssel].min()), 2), max=round(float(sig[ssel].max()), 2),
                         fit_hit_period_bound=bool(f['period'] < 0.55 or f['period'] > 3.9))
    # 대표 주기: 눈에 보이는 휘청 = 상체 접기 진동(upper/fold; 적합·자기상관·FFT 일치). 발(줄) 은 별도로 보고.
    P = float(np.median([ana['upper_deg']['period_s'], ana['fold_deg']['period_s'], ana['upper_deg']['ac_period_s'] or ana['upper_deg']['period_s'], ana['fold_deg']['fft_period_s']]))
    P_foot = float(np.median([ana['foot_x']['period_s'], ana['foot_x']['ac_period_s'] or ana['foot_x']['period_s'], ana['foot_x']['fft_period_s']]))
    print('standing from t=%.2f s; sway analysis:' % t_stand, json.dumps(ana), flush=True)
    P_loop = P if 0.4 <= P <= 3.5 else 2.0
    dur_lo, dur_hi = max(0.6, 0.8 * P_loop), min(3.0, 2.3 * P_loop)

    # ── 루프 구간 탐색: 상태(위치·각도·속도) 정규화 거리, ±2 프레임 창 평균
    def vel(a, i):
        return (a[min(n - 1, i + 1)] - a[max(0, i - 1)]) * FPS / 2

    def state(i):
        return np.array([sm['foot_x'][i], sm['torso_x'][i], sm['head_x'][i], tilt[i], upper[i],
                         vel(sm['foot_x'], i) * 0.15, vel(sm['torso_x'], i) * 0.15, vel(sm['head_x'], i) * 0.15,
                         vel(tilt, i) * 0.15, vel(upper, i) * 0.15])
    S = np.array([state(i) for i in range(n)])
    scale = S[ssel].std(0) + 1e-6
    Sn = S / scale
    cands = []
    for ia in range(stand, n - 1):
        for ib in range(ia + int(dur_lo * FPS), n - 2):
            dur = (ib - ia) / FPS
            if dur < dur_lo or dur > dur_hi:
                continue
            ks = [k for k in (-2, -1, 0, 1, 2) if 0 <= ia + k < n and 0 <= ib + k < n]
            dist = float(np.mean([np.linalg.norm(Sn[ia + k] - Sn[ib + k]) for k in ks]))
            cyc = dur / P_loop
            pen = 0.15 * min(abs(cyc - 1), abs(cyc - 2))
            cands.append((dist + pen, ia, ib, dist))
    if not cands:  # 안전망: 기간 제한 없이
        for ia in range(stand, n - 20):
            for ib in range(ia + 18, n - 2):
                dist = float(np.mean([np.linalg.norm(Sn[ia + k] - Sn[ib + k]) for k in (-2, -1, 0, 1, 2) if ia + k >= 0 and ib + k < n]))
                cands.append((dist, ia, ib, dist))
    cands.sort()
    score, ia, ib, dist = cands[0]
    # 대안: 서로 0.2 s 이상 다른 것들
    alts = []
    for s_, a_, b_, d_ in cands[1:]:
        if all(abs(a_ - x['fa']) > 6 or abs(b_ - x['fb']) > 6 for x in alts):
            alts.append(dict(t_a=round(a_ / FPS, 3), t_b=round(b_ / FPS, 3), score=round(d_, 3), fa=a_, fb=b_))
        if len(alts) >= 5:
            break
    for x in alts:
        x.pop('fa'); x.pop('fb')
    loop = dict(t_a=round(ia / FPS, 3), t_b=round(ib / FPS, 3), score=round(dist, 3),
                frames=[int(ia), int(ib)], duration_s=round((ib - ia) / FPS, 3), cycles=round((ib - ia) / FPS / P_loop, 2), period_used_s=round(P_loop, 3),
                state_mismatch=dict(foot_x_px=round(float(abs(sm['foot_x'][ia] - sm['foot_x'][ib])), 1),
                                    torso_x_px=round(float(abs(sm['torso_x'][ia] - sm['torso_x'][ib])), 1),
                                    head_x_px=round(float(abs(sm['head_x'][ia] - sm['head_x'][ib])), 1),
                                    tilt_deg=round(float(abs(tilt[ia] - tilt[ib])), 2),
                                    upper_deg=round(float(abs(upper[ia] - upper[ib])), 2),
                                    foot_vx_px_s=round(float(abs(vel(sm['foot_x'], ia) - vel(sm['foot_x'], ib))), 1),
                                    head_vx_px_s=round(float(abs(vel(sm['head_x'], ia) - vel(sm['head_x'], ib))), 1)),
                score_note='mean over frames a-2..a+2 vs b-2..b+2 of the Euclidean distance between z-scored state vectors '
                           '[foot_x, torso_x, head_x, tilt, upper, and their velocities*0.15]; 0 = identical, 1 ~ one std of the standing-phase motion',
                alternatives=alts)

    # ── 줄·기둥 요약
    drift_l = post_l_s[-1] - post_l_s[0]; drift_r = post_r_s[-1] - post_r_s[0]
    fy_minus_rope = sm['foot_y'] - rope_yfoot
    rope_summary = dict(
        posts_frame0=dict(left=[round(float(v), 1) for v in post_l_s[0]], right=[round(float(v), 1) for v in post_r_s[0]]),
        posts_last=dict(left=[round(float(v), 1) for v in post_l_s[-1]], right=[round(float(v), 1) for v in post_r_s[-1]]),
        post_drift_px_over_clip=dict(left=[round(float(v), 1) for v in drift_l], right=[round(float(v), 1) for v in drift_r]),
        rope_end_left_mean=[round(float(v), 1) for v in rope_end_l.mean(0)], rope_end_right_mean=[round(float(v), 1) for v in rope_end_r.mean(0)],
        rope_end_left_frame0=[round(float(v), 1) for v in rope_end_l[0]], rope_end_right_frame0=[round(float(v), 1) for v in rope_end_r[0]],
        rope_end_left_last=[round(float(v), 1) for v in rope_end_l[-1]], rope_end_right_last=[round(float(v), 1) for v in rope_end_r[-1]],
        rope_end_minus_post_y=dict(left=round(float((rope_end_l[:, 1] - post_l_s[:, 1]).mean()), 1), right=round(float((rope_end_r[:, 1] - post_r_s[:, 1]).mean()), 1)),
        vertex_mean_standing=[round(float(v), 1) for v in rope_vertex[ssel].mean(0)],
        vertex_minus_foot_standing=[round(float(v), 1) for v in (rope_vertex[ssel] - np.stack([sm['foot_x'][ssel], sm['foot_y'][ssel]], 1)).mean(0)],
        rope_y_under_foot_mean_standing=round(float(rope_yfoot[ssel].mean()), 1),
        foot_y_minus_rope_y=dict(mean=round(float(np.nanmean(fy_minus_rope)), 1), std=round(float(np.nanstd(fy_minus_rope)), 1)),
        slope_left_mean_standing=round(float(rope_l[ssel, 0].mean()), 4), slope_right_mean_standing=round(float(rope_r[ssel, 0].mean()), 4),
        chord_slope_standing=round(float(((rope_end_r[ssel, 1] - rope_end_l[ssel, 1]) / (rope_end_r[ssel, 0] - rope_end_l[ssel, 0])).mean()), 4),
        sag_below_chord_at_vertex_standing=round(float(np.mean([
            rope_vertex[i, 1] - (rope_end_l[i, 1] + (rope_end_r[i, 1] - rope_end_l[i, 1]) * (rope_vertex[i, 0] - rope_end_l[i, 0]) / (rope_end_r[i, 0] - rope_end_l[i, 0]))
            for i in np.nonzero(ssel)[0]])), 1),
        ransac_inliers_mean=[round(float(v), 1) for v in rope_inl.mean(0)],
        method='posts: bamboo mask -> sequential RANSAC lines -> intersection of the two steep pole lines (>20 deg apart), tracked per frame, 5-frame median. '
               'rope: ridge filter on (R+G)/2 -> per-column argmax in a band around foot_y -> RANSAC line on each side of the foot (|x-foot|>45); '
               'rope_end_* = fitted line evaluated at the post-crossing x of that frame; vertex = intersection of the two lines; '
               'image y grows downward, and the whole rope rises to the right in the image (camera roll/perspective), so the deepest image point is the left end; '
               'sag_below_chord_at_vertex is the physically meaningful sag under the performer.',
    )

    mean_foot = [round(float(sm['foot_x'][ssel].mean()), 1), round(float(sm['foot_y'][ssel].mean()), 1)]
    body_len = float(np.mean(np.hypot(sm['head_x'] - sm['foot_x'], sm['head_y'] - sm['foot_y'])[ssel]))
    torso_len = float(np.mean(np.hypot(sm['torso_x'] - sm['foot_x'], sm['torso_y'] - sm['foot_y'])[ssel]))
    interp = {k: [int(i) for i in np.nonzero(~np.isfinite(raw[k]))[0]] for k in keys}

    notes = [
        'Source: presentation/media/jultagi_6s_stab.webm, 1080x1920 portrait, 30 fps, %d frames; all pixel coords are in the full frame (x right, y down).' % n,
        'Performer crop used for tracking: x %d..%d, y %d..%d. Rope/post band: y %d..%d, full width.' % (CX0, CX0 + CW, CY0, CY0 + CH, RY0, RY0 + RH),
        'torso = centroid of dark-blue vest pixels (B>G+5, B>R+15, R<90, G<100), component nearest the previous detection within +-90 px.',
        'foot = per white-clothing component (min(RGB)>110, range<45, B>=G-5) under the vest, the lowest pixel (x = mean of its lowest 6 rows); '
        'the candidate closest to the fitted rope line is taken, so a lifted rear foot during the walk (t<3.6 s) is rejected. '
        'Shoe soles are dark, so foot_y sits on average %.1f px above the fitted rope line (rope.foot_y_minus_rope_y); use rope.y_under_foot for the true contact row.' % np.nanmean(fy_minus_rope),
        'head = centroid of the straw-hat + face pixel component (R>120, R>G+8, G>B+4, R-B>25) nearest the previous head, in a window above the vest. '
        'The fan and sleeves are frequently above the head, so the topmost white pixel is NOT used.',
        'tilt_deg = atan2(torso_x-foot_x, foot_y-torso_y): lower-body lean (positive = torso right of foot in the image). '
        'upper_deg = same for torso->head; fold_deg = upper_deg - tilt_deg (hip fold, negative = head left of the hip line); lean_deg = foot->head.',
        'Smoothing: positions 3-frame median, angles 5-frame median; *_raw / raw{} keep unsmoothed values (failure frames interpolated). Interpolated frames per key in interpolated_frames.',
        'Detection failures (interpolated): torso %s, foot %s, head %s, post_left %s, post_right %s, rope %s.' % tuple(fail[k] or 'none' for k in ('torso', 'foot', 'head', 'post_left', 'post_right', 'rope')),
        'Frames 0..%d (t<%.2f s): performer walks leftwards along the rope (foot_x %.0f -> %.0f px) - the foot alternates between the two feet on the rope. '
        'Standing/sway phase: t>=%.2f s (standing_start_s).' % (stand - 1, t_stand, sm['foot_x'][0], sm['foot_x'][stand], t_stand),
        'Sway analysis on the standing phase: linear trend removed, sinusoid c0+c1 t+A sin(2 pi t/P+phi) with P bounded to [0.5,4] s; '
        'amp = fitted A, p2p_half = (max-min)/2 of the detrended signal, ac/fft periods for reference. The visible sway is a slow (~%.1f s) fold-and-recover of the upper body '
        '(fold_deg swing ~%.0f deg) with a small foot/rope excursion (foot_x ~%.0f px p2p); the foot->torso tilt itself changes only ~%.1f deg.' % (
            P, ana['fold_deg']['p2p_half'] * 2, ana['foot_x']['p2p_half'] * 2, ana['tilt_deg']['p2p_half'] * 2),
        'Loop: minimises the z-scored state distance [foot_x, torso_x, head_x, tilt, upper, velocities] between frames a and b (window +-2 frames), '
        'a>=standing start, duration 0.8..2.3 sway periods, weak preference for 1 or 2 cycles.',
        'The stabilised video still drifts: the left post crossing moves by %s px and the right by %s px between the first and last frame - '
        'use the per-frame post_left/post_right/rope_end arrays when overlaying a model.' % (rope_summary['post_drift_px_over_clip']['left'], rope_summary['post_drift_px_over_clip']['right']),
    ]

    r2 = lambda arr: [[round(float(v), 2) for v in row] for row in arr]
    out = dict(
        source=os.path.relpath(VID, os.path.dirname(V22)), fps=FPS, n=int(n), width=W, height=H,
        t=[round(float(v), 4) for v in t],
        **{k: [round(float(v), 2) for v in sm[k]] for k in keys},
        tilt_deg=[round(float(v), 3) for v in tilt],
        tilt_raw=[round(float(v), 3) for v in tilt_raw],
        upper_deg=[round(float(v), 3) for v in upper], fold_deg=[round(float(v), 3) for v in fold], lean_deg=[round(float(v), 3) for v in lean],
        upper_raw=[round(float(v), 3) for v in upper_raw],
        raw={k: [None if not np.isfinite(v) else round(float(v), 2) for v in raw[k]] for k in keys},
        interpolated_frames=interp, failed_frames=fail,
        vest_area=vest_area.tolist(), legs_area=legs_area.tolist(), head_area=head_area.tolist(),
        foot_rope_dist=[None if not np.isfinite(v) else round(float(v), 1) for v in foot_rope_dist], foot_candidates=foot_ncand.tolist(),
        standing_start_s=round(t_stand, 3), standing_start_frame=int(stand),
        sway=dict(window_s=[round(t_stand, 3), round(float(t[-1]), 3)], dominant_period_s=round(P, 3), period_upper_body_s=round(P, 3), period_foot_s=round(P_foot, 3),
                  period_note='dominant = upper-body fold oscillation (upper_deg/fold_deg; fit, autocorrelation and FFT agree); the foot/rope excursion is slower and smaller (period_foot_s)', **ana),
        loop=loop,
        mean_foot_px=mean_foot, foot_to_torso_px=round(torso_len, 1), foot_to_head_px=round(body_len, 1),
        rope=dict(summary=rope_summary,
                  post_left=r2(post_l_s), post_right=r2(post_r_s),
                  rope_end_left=r2(rope_end_l), rope_end_right=r2(rope_end_r),
                  left_line_ab=[[round(float(a), 5), round(float(b), 2)] for a, b in rope_l],
                  right_line_ab=[[round(float(a), 5), round(float(b), 2)] for a, b in rope_r],
                  vertex=r2(rope_vertex), y_under_foot=[round(float(v), 2) for v in rope_yfoot],
                  line_form='y = a*x + b in full-frame px; left line valid for post_left.x <= x <= vertex.x, right line for vertex.x <= x <= post_right.x'),
        notes=notes,
    )
    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, 'w') as f:
        json.dump(out, f, ensure_ascii=False)
    print('wrote', OUT_JSON, os.path.getsize(OUT_JSON), 'bytes')

    # ── 확인 스트립: 3.0~6.0 s 8 프레임, 평균 발 위치 기준 400x500 crop
    idxs = np.linspace(3.0 * FPS, 6.0 * FPS, 8).round().astype(int)
    cw, ch = 400, 500
    cx = int(mean_foot[0]) - CX0; cy = int(mean_foot[1]) - CY0
    x0 = int(np.clip(cx - cw // 2, 0, CW - cw)); y0 = int(np.clip(cy - ch + 80, 0, CH - ch))
    panels = []
    RED, GRN, YEL, CYA, WHT, MAG = (255, 40, 40), (40, 230, 40), (255, 220, 0), (0, 220, 255), (255, 255, 255), (255, 0, 255)
    for i in idxs:
        p = crop[i, y0:y0 + ch, x0:x0 + cw].copy()
        ox, oy = CX0 + x0, CY0 + y0
        fx, fy = sm['foot_x'][i] - ox, sm['foot_y'][i] - oy
        tx, ty = sm['torso_x'][i] - ox, sm['torso_y'][i] - oy
        hx, hy = sm['head_x'][i] - ox, sm['head_y'][i] - oy
        vx = rope_vertex[i, 0] - ox
        for (a, b), (xa, xb) in (((rope_l[i]), (0, vx)), ((rope_r[i]), (vx, cw - 1))):
            draw_line(p, xa, a * (xa + ox) + b - oy, xb, a * (xb + ox) + b - oy, YEL, 1)
        draw_disc(p, vx, rope_vertex[i, 1] - oy, 3, MAG)
        draw_line(p, fx, fy, fx, fy - 280, WHT, 1)
        draw_line(p, fx, fy, tx, ty, CYA, 3)
        draw_line(p, tx, ty, hx, hy, CYA, 3)
        draw_disc(p, fx, fy, 5, RED); draw_disc(p, tx, ty, 5, GRN); draw_disc(p, hx, hy, 5, YEL)
        p[:40] = (p[:40] * 0.35).astype(np.uint8)
        draw_text(p, 4, 4, 't=%.2f tilt=%+.1f' % (i / FPS, tilt[i]), WHT, 2)
        draw_text(p, 4, 22, 'upper=%+.1f fold=%+.1f' % (upper[i], fold[i]), WHT, 2)
        p[:, -2:] = 255
        panels.append(p)
    write_png(os.path.join(CHECK_DIR, 'track_check.png'), np.concatenate(panels, 1))

    # ── 플롯(numpy 래스터): 4 패널
    PW, PH, ML, MR, MT, PB, GAP = 1200, 1000, 90, 20, 30, 40, 40
    npan = 4
    ph = (PH - MT - PB - (npan - 1) * GAP) // npan
    img = np.full((PH, PW, 3), 250, np.uint8)
    panels = [('foot_x [px]', [('foot_x', sm['foot_x'], raw['foot_x'], (200, 30, 30)), ('rope vertex x', rope_vertex[:, 0], None, (230, 150, 0))]),
              ('torso_x / head_x [px]', [('torso_x', sm['torso_x'], raw['torso_x'], (30, 140, 30)), ('head_x', sm['head_x'], raw['head_x'], (200, 120, 0))]),
              ('tilt_deg (foot-torso)', [('tilt', tilt, tilt_raw, (30, 60, 200))]),
              ('upper_deg / fold_deg', [('upper', upper, upper_raw, (120, 30, 160)), ('fold', fold, None, (0, 150, 150))])]
    tmax = t[-1]
    for k, (label, ser) in enumerate(panels):
        top = MT + k * (ph + GAP); bot = top + ph
        img[top:bot, ML:PW - MR] = 255
        allv = np.concatenate([s[1] for s in ser])
        lo, hi = np.nanmin(allv) - 2, np.nanmax(allv) + 2
        lo, hi = lo - (hi - lo) * 0.08, hi + (hi - lo) * 0.08

        def X(tt): return ML + (PW - MR - ML) * tt / tmax
        def Y(v): return bot - (v - lo) / (hi - lo) * ph
        for tt in np.arange(0, tmax + 0.01, 0.5):
            draw_line(img, X(tt), top, X(tt), bot, (225, 225, 225), 1)
            draw_text(img, int(X(tt)) - 8, bot + 6, '%.1f' % tt, (60, 60, 60), 2)
        stp = 20 if hi - lo > 80 else (10 if hi - lo > 40 else (5 if hi - lo > 15 else 2))
        for v in np.arange(np.ceil(lo / stp) * stp, hi, stp):
            draw_line(img, ML, Y(v), PW - MR, Y(v), (225, 225, 225), 1)
            draw_text(img, 6, int(Y(v)) - 6, '%6.0f' % v, (60, 60, 60), 2)
        xa, xb = int(X(loop['t_a'])), int(X(loop['t_b']))
        img[top:bot, xa:xb] = (img[top:bot, xa:xb] * 0.9 + np.array([255, 240, 190]) * 0.1).astype(np.uint8)
        draw_line(img, xa, top, xa, bot, (230, 150, 0), 1); draw_line(img, xb, top, xb, bot, (230, 150, 0), 1)
        draw_line(img, X(t_stand), top, X(t_stand), bot, (150, 150, 150), 1)
        for name, ysm, yraw, col in ser:
            if yraw is not None:
                for j in range(n - 1):
                    if np.isfinite(yraw[j]) and np.isfinite(yraw[j + 1]):
                        draw_line(img, X(t[j]), Y(yraw[j]), X(t[j + 1]), Y(yraw[j + 1]), (190, 190, 190), 1)
            for j in range(n - 1):
                draw_line(img, X(t[j]), Y(ysm[j]), X(t[j + 1]), Y(ysm[j + 1]), col, 3 if name in ('foot_x', 'torso_x', 'tilt', 'upper') else 1)
        xx = ML + 8
        for name, _, _, col in ser:
            draw_text(img, xx, top + 6, name, col, 2); xx += 6 * 2 * (len(name) + 2)
        img[top - 1, ML:PW - MR] = 120; img[bot, ML:PW - MR] = 120; img[top:bot, ML - 1] = 120; img[top:bot, PW - MR] = 120
    draw_text(img, ML, PH - 22, 't [s]   grey=raw  standing from %.2f s  loop [%.2f, %.2f] s  period %.2f s' % (t_stand, loop['t_a'], loop['t_b'], P), (40, 40, 40), 2)
    write_png(os.path.join(CHECK_DIR, 'track_plot.png'), img)

    # ── 리포트
    ok = lambda a: a[a > 0]
    print('\n=== jultagi sway tracking report ===')
    print('frames %d, fps %g, duration %.2f s; standing/sway phase from t=%.2f s' % (n, FPS, t[-1], t_stand))
    print('detection: vest area mean %.0f px (min %d); legs area mean %.0f (min %d); head area mean %.0f (min %d)'
          % (ok(vest_area).mean(), ok(vest_area).min(), ok(legs_area).mean(), ok(legs_area).min(), ok(head_area).mean(), ok(head_area).min()))
    print('   failures (interpolated): torso=%s foot=%s head=%s post_left=%s post_right=%s rope=%s'
          % tuple(fail[k] or 'none' for k in ('torso', 'foot', 'head', 'post_left', 'post_right', 'rope')))
    print('   foot-to-rope distance: mean %.1f px, |max| %.1f px; frames with >1 foot candidate: %d'
          % (np.nanmean(foot_rope_dist), np.nanmax(np.abs(foot_rope_dist)), int((foot_ncand > 1).sum())))
    print('   raw-vs-smooth |diff| median: ' + ', '.join('%s %.2f' % (k, np.nanmedian(np.abs(filled[k] - sm[k]))) for k in keys)
          + ', tilt %.2f deg; |diff| max tilt %.2f deg' % (np.nanmedian(np.abs(tilt_raw - tilt)), np.nanmax(np.abs(tilt_raw - tilt))))
    print('foot_x(t): t=0 %.1f -> t=3 %.1f -> stand %.1f -> end %.1f ; foot_y standing mean %.1f (std %.1f)' %
          (sm['foot_x'][0], sm['foot_x'][90], sm['foot_x'][stand], sm['foot_x'][-1], sm['foot_y'][ssel].mean(), sm['foot_y'][ssel].std()))
    print('sway (t>=%.2f s):  dominant (upper-body fold) period %.2f s ; foot/rope period %.2f s' % (t_stand, P, P_foot))
    for k, v in ana.items():
        print('   %-9s fitP %.2f s%s (ac %s, fft %.2f)  amp %.2f  p2p/2 %.2f  resid %.2f  range [%.1f, %.1f] mean %.1f' %
              (k, v['period_s'], '*' if v['fit_hit_period_bound'] else ' ', v['ac_period_s'], v['fft_period_s'], v['amp'], v['p2p_half'], v['rms_resid'], v['min'], v['max'], v['mean']))
    print('   (* = sinusoid fit hit the period bound: trend-dominated, period not meaningful)')
    print('loop: t_a=%.3f t_b=%.3f (frames %d..%d, %.2f s = %.2f cycles of %.2f s) score=%.3f mismatch %s' %
          (loop['t_a'], loop['t_b'], ia, ib, loop['duration_s'], loop['cycles'], P_loop, loop['score'], loop['state_mismatch']))
    print('   alternatives:', loop['alternatives'][:4])
    print('mean foot (standing): %s ; foot->torso %.1f px ; foot->head %.1f px' % (mean_foot, torso_len, body_len))
    rs = rope_summary
    print('posts (crossings): frame0 L %s R %s ; last L %s R %s ; drift L %s R %s' %
          (rs['posts_frame0']['left'], rs['posts_frame0']['right'], rs['posts_last']['left'], rs['posts_last']['right'],
           rs['post_drift_px_over_clip']['left'], rs['post_drift_px_over_clip']['right']))
    print('rope ends (fitted line at post x): frame0 L %s R %s ; last L %s R %s ; rope_end - post y: %s' %
          (rs['rope_end_left_frame0'], rs['rope_end_right_frame0'], rs['rope_end_left_last'], rs['rope_end_right_last'], rs['rope_end_minus_post_y']))
    print('rope under performer (standing): vertex %s (vertex-foot %s) ; rope y under foot %.1f ; foot_y-rope_y %.1f +- %.1f ; slopes L %.4f R %.4f chord %.4f ; sag below chord at vertex %.1f px'
          % (rs['vertex_mean_standing'], rs['vertex_minus_foot_standing'], rs['rope_y_under_foot_mean_standing'], rs['foot_y_minus_rope_y']['mean'],
             rs['foot_y_minus_rope_y']['std'], rs['slope_left_mean_standing'], rs['slope_right_mean_standing'], rs['chord_slope_standing'], rs['sag_below_chord_at_vertex_standing']))
    print('   RANSAC inliers/samples mean (L in, L n, R in, R n):', rs['ransac_inliers_mean'])
    print('outputs:', OUT_JSON, os.path.join(CHECK_DIR, 'track_check.png'), os.path.join(CHECK_DIR, 'track_plot.png'))
    print('\n   t     foot_x  foot_y  torso_x  head_x  tilt   upper  fold   ropeY  postL(x,y)      postR(x,y)')
    for i in range(int(3.0 * FPS), n, 3):
        print('  %5.2f  %6.1f  %6.1f  %7.1f  %6.1f  %+5.1f  %+5.1f  %+5.1f  %6.1f  (%5.1f,%6.1f)  (%5.1f,%6.1f)' %
              (t[i], sm['foot_x'][i], sm['foot_y'][i], sm['torso_x'][i], sm['head_x'][i], tilt[i], upper[i], fold[i], rope_yfoot[i],
               post_l_s[i, 0], post_l_s[i, 1], post_r_s[i, 0], post_r_s[i, 1]))


if __name__ == '__main__':
    main()
