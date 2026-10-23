"""铁片盘面定位与缺陷候选提取（纯 OpenCV，不依赖 ROS，可离线单测）。

场景假设（与仿真 / 赛场一致）：
  俯视相机 -> 库位里有一个深色凹槽 -> 圆形铁片躺在凹槽内 -> 盘面上有一处缺陷。

为什么不用全图 HoughCircles（坑）：
  台面亮度与铁片只差几级灰度，全图 Hough 会锁到台面边缘/夹爪，
  实测同一批图里圆心能在 (220,240) 与 (319,239) 之间乱跳。
  改用「先找暗凹槽 -> 只在凹槽 ROI 内拟合圆」，圆心稳定在 ±3 px。

为什么 Hough 之后还要精修（坑 18）：
  HoughCircles 的输出落在 dp 网格上（dp=1.2 时坐标是 1.2 px 的整数倍），
  同一台相机、同一高度下逐帧半径会在 80.3~87.4 px 之间跳、圆心 cy 差 7 px。
  而 frac / rad 全部按盘半径归一化 —— 半径小 9%，压在盘缘的缺角就会被 0.93r 的
  掩膜切掉大半（实测 frac 0.072 -> 0.030），阈值只能往最坏情况让，余量全没了。
  所以 find_disc_global 末尾统一调 refine_circle()：径向边缘扫描 + Kasa 拟合，
  实测组内半径抖动从 2.30 px 降到 0.10~0.49 px，四类特征的帧间散差随之塌到 1e-3 量级。

坐标全部以「盘半径 r」归一化，因此对相机高度不敏感。
"""

import cv2
import numpy as np


class DetectorParams(object):
    """可调参数集中在这里；默认值是在 640x480 / 盘径 80 mm 下整定的。"""

    def __init__(self, **kw):
        # 凹槽：不接触图像边框的最大暗连通域，低于该面积视为噪声
        self.pocket_min_area = 5000
        # ROI 内缩像素（避开凹槽与台面的过渡带）
        self.pocket_margin = 8
        # 圆半径搜索范围（占 ROI 短边的比例）
        self.radius_min_frac = 0.28
        self.radius_max_frac = 0.48
        # Hough 参数，按强到弱依次尝试
        self.hough_param1 = 90
        self.hough_param2_seq = (55, 45, 35, 28)
        # 缺陷暗度阈值：比盘面参考灰度低多少算「暗」
        self.dark_delta = 30
        # 盘面参考灰度取样半径（占 r 的比例，避开盘缘倒角）
        self.level_frac = 0.55
        # 缺陷分析域半径（占 r 的比例，必须 < 1：盘缘外的凹槽壁会污染）
        self.roi_frac = 0.93
        # 连通域最小面积（像素）
        self.min_comp_area = 120
        # 形态学开运算核
        self.open_ksize = 3
        # ---- 整图找盘（主路径）----
        # 盘半径搜索范围（占图像短边比例）
        self.disc_rmin_frac = 0.09
        self.disc_rmax_frac = 0.26
        # 盘内平均灰度 - 盘外环平均灰度 的下限：低于它认为不是盘（是台面/结构）
        self.disc_contrast_min = 12.0
        # 离图像中心越远扣分越重，用于在同分候选里优先取近中心的
        self.disc_center_penalty = 80.0
        self.disc_hough_param2_seq = (60, 50, 42, 34)
        # ---- 圆精修（refine_circle）----
        # 径向扫描的射线数 / 半径采样点数
        self.refine_rays = 180
        self.refine_radii = 40
        # 扫描半径范围（相对 Hough 半径）
        self.refine_band = (0.80, 1.20)
        # 单条射线的边缘梯度峰值下限（相邻采样点的灰阶差），低于它视为该方向被遮挡
        self.refine_grad_min = 6.0
        # 精修结果被采纳的条件：内点占比、半径相对变化、残差（像素）
        self.refine_inlier_min = 0.45
        self.refine_max_dr = 0.12
        self.refine_max_resid = 1.5
        # ---- 近黑核心（区分「孔洞」与「缺角」）----
        # 孔是通透的，孔口像素接近黑；缺角只是表面缺了一块料，亮度跟其它缺陷同级。
        # 实测（24 帧）：孔洞斑内 < level-120 的像素占 0.23~0.75，其余三类全是 0.000。
        self.core_delta = 120.0
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError("未知参数: {}".format(k))
            setattr(self, k, v)


def find_pocket(gray, p):
    """返回深色凹槽的外接框 dict(x,y,w,h)，找不到返回 None。

    判据：「暗」用 Otsu 自动取阈，不写死灰度，适应不同曝光。
    排除接触图像边框的暗区（台面阴影 / 画面外的暗角）。
    """
    h, w = gray.shape[:2]
    t, _ = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    n, _, stats, _ = cv2.connectedComponentsWithStats((gray < t).astype(np.uint8), 8)
    best = None
    for i in range(1, n):
        x0, y0, bw, bh, area = stats[i]
        if x0 <= 1 or y0 <= 1 or x0 + bw >= w - 1 or y0 + bh >= h - 1:
            continue
        if area < p.pocket_min_area:
            continue
        if best is None or area > best[0]:
            best = (area, x0, y0, bw, bh)
    if best is None:
        return None
    return {"x": int(best[1]), "y": int(best[2]), "w": int(best[3]), "h": int(best[4])}


def find_plate(gray, pocket, p):
    """在凹槽 ROI 内拟合铁片圆，返回 dict(cx,cy,r)；失败返回 None。"""
    m = p.pocket_margin
    x0 = max(0, pocket["x"] + m)
    y0 = max(0, pocket["y"] + m)
    x1 = min(gray.shape[1], pocket["x"] + pocket["w"] - m)
    y1 = min(gray.shape[0], pocket["y"] + pocket["h"] - m)
    if x1 - x0 < 40 or y1 - y0 < 40:
        return None
    roi = cv2.GaussianBlur(gray[y0:y1, x0:x1], (5, 5), 1.2)
    short = min(roi.shape[:2])
    rmin = max(8, int(p.radius_min_frac * short))
    rmax = max(rmin + 4, int(p.radius_max_frac * short))
    for p2 in p.hough_param2_seq:
        c = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, dp=1.0, minDist=int(short * 0.6),
                             param1=p.hough_param1, param2=p2,
                             minRadius=rmin, maxRadius=rmax)
        if c is None:
            continue
        cx, cy, r = c[0][0]
        return {"cx": float(cx + x0), "cy": float(cy + y0), "r": float(r)}
    return None


def extract_defects(gray, plate, p):
    """返回 (盘面参考灰度 level, 候选连通域列表)。

    每个候选包含归一化特征：
      frac  占盘面积比（= area / pi r^2）
      circ  圆度 4piA/P^2（1 = 完美圆）
      elong minAreaRect 长宽比（划痕会很大）
      rad   质心到盘心距离 / r（缺角贴近 1，盘内缺陷接近 0）
      mean  该连通域的平均灰度
      drop  盘面灰度 - mean（越大越黑）
    """
    cx, cy, r = plate["cx"], plate["cy"], plate["r"]
    ci = (int(round(cx)), int(round(cy)))

    inner = np.zeros_like(gray)
    cv2.circle(inner, ci, max(2, int(r * p.level_frac)), 255, -1)
    level = float(np.median(gray[inner > 0]))

    roi = np.zeros_like(gray)
    cv2.circle(roi, ci, max(2, int(r * p.roi_frac)), 255, -1)
    dark = ((gray < level - p.dark_delta) & (roi > 0)).astype(np.uint8) * 255
    if p.open_ksize > 0:
        k = np.ones((p.open_ksize, p.open_ksize), np.uint8)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, k)

    n, labels, stats, cents = cv2.connectedComponentsWithStats(dark, 8)
    plate_area = np.pi * r * r
    comps = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < p.min_comp_area:
            continue
        mask = (labels == i).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        cnt = max(contours, key=cv2.contourArea)
        perim = cv2.arcLength(cnt, True)
        circ = 4.0 * np.pi * area / (perim * perim) if perim > 0 else 0.0
        rw, rh = cv2.minAreaRect(cnt)[1]
        rect_area = max(1.0, float(rw * rh))
        # fill = 像素面积 / 最小外接矩形面积。直线/圆斑都能填满自己的外接框（≈0.8~0.95），
        # 折线（破损的 ∧ 形）只有 ≈0.3~0.5 —— 这是把「破损」和「划痕/污渍」分开的主特征。
        fill = float(area) / rect_area
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        solidity = float(area) / hull_area if hull_area > 1.0 else 1.0
        mean = float(gray[labels == i].mean())
        # 近黑像素占比：只在这个连通域内统计（不看 bbox，免得把周围亮台面算进来）
        comp_px = gray[labels == i]
        core_frac = float((comp_px < level - p.core_delta).mean())
        comps.append({
            "area": area,
            "frac": area / plate_area,
            "circ": float(circ),
            "elong": float(max(rw, rh) / max(1.0, min(rw, rh))),
            "rad": float(np.hypot(cents[i][0] - cx, cents[i][1] - cy) / r),
            "fill": fill,
            "solidity": solidity,
            "mean": mean,
            "drop": level - mean,
            "core_frac": core_frac,
            # 外接框只给可视化用（make_atlas.py 画缺陷框）；判据不读它
            "bbox": cv2.boundingRect(cnt),
        })
    comps.sort(key=lambda c: -c["area"])
    return level, comps


def _kasa_fit(px, py):
    """代数最小二乘圆拟合（Kasa）：返回 (cx, cy, r)。"""
    a = np.column_stack([2.0 * px, 2.0 * py, np.ones_like(px)])
    b = px * px + py * py
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy = float(sol[0]), float(sol[1])
    r = float(np.sqrt(max(1e-9, sol[2] + cx * cx + cy * cy)))
    return cx, cy, r


def refine_circle(gray, plate, p):
    """把 Hough 给的整像素圆精修成亚像素圆：径向边缘扫描 + Kasa 拟合。

    为什么需要（坑 18）：
      `HoughCircles(dp=1.2)` 的输出是 1.2 px 网格上的整数，同一个静止场景逐帧跑
      半径会在 80.3–87.4 px 之间跳（±4%），圆心 cy 能差 7 px。而下游所有几何量
      （frac / rad）都按盘半径归一化，半径偏小 9% 就会把压在盘缘的缺角切掉大半
      （实测 frac 0.072 → 0.030），判据阈值根本没法设稳。
      这里沿 N 条射线找「亮盘 -> 暗底」的边缘点，再用最小二乘拟合，半径抖动降到
      亚像素级；被夹指挡住的方向按梯度峰值强度剔除。

    接受条件（不满足就退回 Hough 原值，宁可不改也不改坏）：
      内点占比 >= refine_inlier_min、|dr|/r <= refine_max_dr、残差 <= refine_max_resid。

    返回 dict(cx, cy, r, refined, inliers, resid)。
    """
    cx0, cy0, r0 = float(plate["cx"]), float(plate["cy"]), float(plate["r"])
    h, w = gray.shape[:2]
    lo, hi = p.refine_band
    radii = np.linspace(r0 * lo, r0 * hi, int(p.refine_radii))
    thetas = np.linspace(0.0, 2.0 * np.pi, int(p.refine_rays), endpoint=False)

    xs = cx0 + np.outer(np.cos(thetas), radii)
    ys = cy0 + np.outer(np.sin(thetas), radii)
    samp = cv2.remap(gray.astype(np.float32), xs.astype(np.float32), ys.astype(np.float32),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    # 沿半径方向的梯度：只有「由亮转暗」的边缘才算盘缘（正梯度，取负号）
    grad = -np.diff(samp, axis=1)
    idx = np.argmax(grad, axis=1)
    peak = grad[np.arange(grad.shape[0]), idx]

    keep = peak >= max(p.refine_grad_min, 0.5 * float(np.median(peak)))
    if keep.sum() < max(8, int(p.refine_inlier_min * len(thetas))):
        return {"cx": cx0, "cy": cy0, "r": r0, "refined": False, "inliers": 0, "resid": 0.0}

    r_edge = radii[idx] + 0.5 * (radii[1] - radii[0])
    ex = cx0 + r_edge * np.cos(thetas)
    ey = cy0 + r_edge * np.sin(thetas)

    # 两轮离群剔除后再拟合（夹指边缘、盘面缺陷都可能贡献假边缘点）
    cx, cy, r = cx0, cy0, r0
    sel = keep.copy()
    for _ in range(2):
        if sel.sum() < 8:
            break
        cx, cy, r = _kasa_fit(ex[sel], ey[sel])
        resid = np.abs(np.hypot(ex - cx, ey - cy) - r)
        tol = max(1.0, 2.0 * float(np.median(resid[sel])))
        sel = keep & (resid <= tol)

    resid_all = float(np.median(np.abs(np.hypot(ex - cx, ey - cy) - r)[sel])) if sel.any() else 99.0
    inlier_ratio = float(sel.sum()) / float(len(thetas))
    if (inlier_ratio < p.refine_inlier_min
            or abs(r - r0) > p.refine_max_dr * r0
            or resid_all > p.refine_max_resid):
        return {"cx": cx0, "cy": cy0, "r": r0, "refined": False,
                "inliers": round(inlier_ratio, 2), "resid": round(resid_all, 2)}
    return {"cx": cx, "cy": cy, "r": r, "refined": True,
            "inliers": round(inlier_ratio, 2), "resid": round(resid_all, 2)}


def find_disc_global(gray, p):
    """在整图上直接找盘（主路径）：Hough 圆 + 「内亮外暗」环对比筛选。

    为什么不再「先找凹槽、要求凹槽完整入镜」：
      相机不一定正对库位中心（实拍中零件常常偏在半边画面），一旦凹槽被画面边缘切掉，
      整条链路就判「未定位」。而盘本身是个稳定的亮圆，直接被找到就没这问题。
      批量验证（12 张样本 + 4 张偏置/遮挡帧）全部命中，圆心与半径都合理。

    返回 dict(cx,cy,r,contrast) 或 None。
    """
    h, w = gray.shape[:2]
    blur = cv2.GaussianBlur(gray, (5, 5), 1.2)
    rmin = max(6, int(p.disc_rmin_frac * h))
    rmax = max(rmin + 4, int(p.disc_rmax_frac * h))
    best = None
    for p2 in p.disc_hough_param2_seq:
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1.2,
                                   minDist=int(h * 0.25), param1=90, param2=p2,
                                   minRadius=rmin, maxRadius=rmax)
        if circles is None:
            continue
        for x, y, r in circles[0]:
            x, y, r = float(x), float(y), float(r)
            inner = np.zeros_like(gray)
            cv2.circle(inner, (int(x), int(y)), max(2, int(r * 0.70)), 255, -1)
            ring = np.zeros_like(gray)
            cv2.circle(ring, (int(x), int(y)), int(r * 1.35), 255, -1)
            cv2.circle(ring, (int(x), int(y)), int(r * 1.02), 0, -1)
            if (inner > 0).sum() < 50 or (ring > 0).sum() < 50:
                continue
            contrast = float(gray[inner > 0].mean()) - float(gray[ring > 0].mean())
            if contrast < p.disc_contrast_min:
                continue
            dist = float(np.hypot(x - w / 2.0, y - h / 2.0) / min(h, w))
            score = contrast - p.disc_center_penalty * dist
            if best is None or score > best[0]:
                best = (score, x, y, r, contrast)
        if best is not None:
            break
    if best is None:
        return None
    plate = {"cx": best[1], "cy": best[2], "r": best[3], "contrast": best[4]}
    # Hough 只给到 1.2 px 网格；再精修一步，否则半径抖动会污染所有归一化判据
    plate.update(refine_circle(gray, plate, p))
    return plate


def detect(gray, p=None):
    """一步到位：图像 -> 定位 + 候选。返回 dict 或 None（定位失败）。

    路径：先在整图找盘（稳），失败再退回「凹槽 ROI 内拟合圆」（画面干净时更准）。
    入参同时接受灰度图和 BGR 图：离线脚本传灰度、ROS 节点一开始传 BGR，
    曾因此在 cv2.threshold(THRESH_OTSU) 里抛 "src_type is 16 (CV_8UC3)" 把节点打挂。
    """
    p = p or DetectorParams()
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)

    plate = find_disc_global(gray, p)
    if plate is None:
        pocket = find_pocket(gray, p)
        plate = find_plate(gray, pocket, p) if pocket else None
        if plate is not None:
            # 兜底路径同样要精修：下游判据只认归一化量，半径精度决定一切
            plate.update(refine_circle(gray, plate, p))
    if plate is None:
        return None

    level, comps = extract_defects(gray, plate, p)
    return {"plate": plate, "level": level, "comps": comps, "gray": gray}
