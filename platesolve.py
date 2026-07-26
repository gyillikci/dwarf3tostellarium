#!/usr/bin/env python3
"""
platesolve.py — pure-Python lost-in-space plate solver for dwarf3tostellarium.

A Python port of the OPNAV star-identification + attitude-determination pipeline
from SONIC (opnavlab/sonic), mirroring its math so SONIC (in MATLAB) can serve as
a validation oracle. See OPNAV_STUDY.md for the design rationale.

Pipeline (image -> RA/Dec of boresight), same as SONIC:

    image ──(Centroider.COB)──▶ star centroids (pixels)
          ──(Camera.Kinv + undistort)──▶ line-of-sight unit vectors (camera frame)
          ──(StarId.interstarAngle)──▶ matches to a star catalog + attitude
          ──(Attitude.solveWahbasProblem / SVD)──▶ world→camera DCM
          ──(3rd row of DCM -> RA/Dec)──▶ boresight pointing

Dependencies:
    - numpy            REQUIRED (geometry + solver core).
    - opencv (cv2) OR scipy.ndimage   OPTIONAL, only for centroiding a real image.
      The geometry/solver core needs neither; you can pass line-of-sight vectors
      or centroids directly. Absent deps degrade gracefully, like skyfield/bleak
      elsewhere in this repo.

Run `python3 platesolve.py` for a self-contained synthetic end-to-end test that
needs only numpy (no Hipparcos data, no real frame).

This module is made available under the same license as the repository.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

# ── Optional deps, resolved lazily so importing this module never fails ──────
try:
    import cv2  # noqa: F401
    _HAVE_CV2 = True
except Exception:
    _HAVE_CV2 = False

try:
    from scipy import ndimage as _ndimage  # noqa: F401
    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False


# ─────────────────────────────────────────────────────────────────────────────
# Spherical / vector helpers  (mirror sonic.SphereCoords)
# ─────────────────────────────────────────────────────────────────────────────
def radec_to_unit(ra_rad, dec_rad) -> np.ndarray:
    """(RA, Dec) in radians -> 3xN ICRF unit vectors. Vectorized."""
    ra = np.atleast_1d(np.asarray(ra_rad, float))
    dec = np.atleast_1d(np.asarray(dec_rad, float))
    return np.vstack((np.cos(dec) * np.cos(ra),
                      np.cos(dec) * np.sin(ra),
                      np.sin(dec)))


def unit_to_radec(u) -> tuple[np.ndarray, np.ndarray]:
    """3xN unit vectors -> (RA in [0,2pi), Dec in [-pi/2,pi/2]), radians."""
    u = np.asarray(u, float)
    if u.ndim == 1:
        u = u[:, None]
    n = u / np.linalg.norm(u, axis=0, keepdims=True)
    ra = np.mod(np.arctan2(n[1], n[0]), 2 * math.pi)
    dec = np.arcsin(np.clip(n[2], -1.0, 1.0))
    return ra, dec


def _normalize_cols(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v, axis=0, keepdims=True)


# ─────────────────────────────────────────────────────────────────────────────
# Attitude  (mirror sonic.Attitude.solveWahbasProblem — Markley's SVD method)
# ─────────────────────────────────────────────────────────────────────────────
def wahba_svd(u_cam: np.ndarray, u_world: np.ndarray) -> np.ndarray:
    """
    Solve Wahba's problem for the world->camera DCM T such that u_cam ≈ T @ u_world.

    Identical to sonic.Attitude.solveWahbasProblem:
        [U,~,V] = svd(u_cam * u_world');  M = diag([1,1,det(U)det(V)]);  T = U*M*V'

    Inputs are 3xN arrays of matched unit direction pairs (N >= 2).
    """
    u_cam = np.asarray(u_cam, float)
    u_world = np.asarray(u_world, float)
    if u_cam.shape[1] < 2 or u_cam.shape != u_world.shape:
        raise ValueError("wahba_svd needs >=2 matching 3xN direction pairs")
    B = u_cam @ u_world.T
    U, _, Vt = np.linalg.svd(B)
    M = np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(Vt)])
    return U @ M @ Vt


def attitude_angle_between(T1: np.ndarray, T2: np.ndarray) -> float:
    """Rotation angle (rad) between two DCMs."""
    R = T1 @ T2.T
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return math.acos(c)


# ─────────────────────────────────────────────────────────────────────────────
# Camera model  (mirror sonic.Camera + Pinhole / BrownConrady distortion)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class CameraModel:
    """
    Framing-camera model: intrinsics K plus an optional Brown-Conrady distortion
    (OpenCV's model). Canonical camera frame: +z boresight, +x right, +y down;
    pixel (u,v) origin top-left.
    """
    d_x: float
    d_y: float
    u_p: float
    v_p: float
    alpha: float = 0.0
    # Brown-Conrady / OpenCV distortion (all zero == pinhole passthrough):
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0
    res: Optional[tuple[int, int]] = None  # (rows, cols) for reference

    # --- constructors -------------------------------------------------------
    @classmethod
    def from_fov(cls, fov_deg: float, res_wh: tuple[int, int],
                 direction: str = "h", **dist) -> "CameraModel":
        """
        Build from full-angle FOV (degrees) + resolution (width, height) in px,
        assuming square pixels and no shear (mirrors Camera.constructFromFOV).
        `direction` = 'h' (horizontal FOV) or 'v' (vertical FOV).
        """
        w, h = res_wh
        fov = math.radians(fov_deg)
        major = w if direction.lower() == "h" else h
        d = major / (2.0 * math.tan(fov / 2.0))
        return cls(d_x=d, d_y=d, u_p=w / 2.0, v_p=h / 2.0,
                   res=(h, w), **dist)

    # --- intrinsics ---------------------------------------------------------
    @property
    def K(self) -> np.ndarray:
        return np.array([[self.d_x, self.alpha, self.u_p],
                         [0.0, self.d_y, self.v_p],
                         [0.0, 0.0, 1.0]])

    @property
    def Kinv(self) -> np.ndarray:
        dxdy = self.d_x * self.d_y
        return np.array([
            [1.0 / self.d_x, -self.alpha / dxdy,
             (self.alpha * self.v_p - self.d_y * self.u_p) / dxdy],
            [0.0, 1.0 / self.d_y, -self.v_p / self.d_y],
            [0.0, 0.0, 1.0]])

    @property
    def hfov_rad(self) -> float:
        return 2.0 * math.atan(self.u_p / self.d_x)

    @property
    def vfov_rad(self) -> float:
        return 2.0 * math.atan(self.v_p / self.d_y)

    @property
    def diag_fov_rad(self) -> float:
        return 2.0 * math.atan(math.hypot(self.u_p, self.v_p) / self.d_x)

    @property
    def ifov_rad(self) -> float:
        """Instantaneous FOV (rad/pixel), x direction."""
        return 1.0 / self.d_x

    def _has_distortion(self) -> bool:
        return any(abs(c) > 0 for c in (self.k1, self.k2, self.p1, self.p2, self.k3))

    # --- forward (world ray -> pixel), used for synthesis -------------------
    def _distort(self, xy: np.ndarray) -> np.ndarray:
        """Apply Brown-Conrady to 2xN normalized image-plane coords."""
        x, y = xy[0], xy[1]
        r2 = x * x + y * y
        radial = 1 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 ** 3
        xd = x * radial + 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
        yd = y * radial + self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
        return np.vstack((xd, yd))

    def _undistort(self, xy: np.ndarray, iters: int = 20) -> np.ndarray:
        """Invert Brown-Conrady (OpenCV-style fixed-point iteration)."""
        if not self._has_distortion():
            return xy
        xd, yd = xy[0], xy[1]
        x, y = xd.copy(), yd.copy()
        for _ in range(iters):
            r2 = x * x + y * y
            radial = 1 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 ** 3
            dx = 2 * self.p1 * x * y + self.p2 * (r2 + 2 * x * x)
            dy = self.p1 * (r2 + 2 * y * y) + 2 * self.p2 * x * y
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        return np.vstack((x, y))

    def pixels_to_los(self, pix: np.ndarray) -> np.ndarray:
        """
        2xN pixel coords -> 3xN unit line-of-sight vectors in the camera frame.
        Applies Kinv then undistorts (mirrors the reverse of Camera.synthImage).
        """
        pix = np.asarray(pix, float)
        if pix.ndim == 1:
            pix = pix[:, None]
        homog = np.vstack((pix, np.ones((1, pix.shape[1]))))
        norm = self.Kinv @ homog                       # distorted normalized coords
        undist = self._undistort(norm[:2])             # -> ideal normalized coords
        rays = np.vstack((undist, np.ones((1, undist.shape[1]))))
        return _normalize_cols(rays)

    def los_to_pixels(self, los: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        3xN camera-frame directions -> 2xN pixel coords (forward projection).
        Returns (pixels, in_front_mask). Used for synthesis / reprojection.
        """
        los = np.asarray(los, float)
        z = los[2]
        in_front = z > 1e-9
        xy = np.vstack((los[0] / z, los[1] / z))
        xy_d = self._distort(xy)
        homog = np.vstack((xy_d, np.ones((1, xy_d.shape[1]))))
        pix = self.K @ homog
        return pix[:2], in_front


# DWARF 3 camera presets (from roi_gui.py: WIDE_FOV_DEG=60, TELE_FOV_DEG=3.4,
# wide stream 1920x1080). FOV/resolution are nominal — REFINE BY CALIBRATION,
# and add real Brown-Conrady terms, before trusting a solve to the frame edges.
def dwarf3_wide_camera(res_wh=(1920, 1080)) -> CameraModel:
    return CameraModel.from_fov(60.0, res_wh, "h")


def dwarf3_tele_camera(res_wh=(1920, 1080)) -> CameraModel:
    return CameraModel.from_fov(3.4, res_wh, "h")


# ─────────────────────────────────────────────────────────────────────────────
# Star catalog  (mirror sonic.StarCatalog / Hipparcos: unit vectors + magnitude)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class StarCatalog:
    """
    A star catalog reduced to what identification needs: ICRF unit vectors and
    visual magnitudes. This prototype omits proper motion/parallax (SONIC's
    5-parameter model) — for a night's imaging the error is negligible; add
    epoch propagation later if sub-arcsecond accuracy is needed.
    """
    ra_rad: np.ndarray
    dec_rad: np.ndarray
    vmag: np.ndarray
    ids: Optional[np.ndarray] = None
    unit: np.ndarray = field(init=False)

    def __post_init__(self):
        self.ra_rad = np.asarray(self.ra_rad, float).ravel()
        self.dec_rad = np.asarray(self.dec_rad, float).ravel()
        self.vmag = np.asarray(self.vmag, float).ravel()
        if self.ids is None:
            self.ids = np.arange(len(self.ra_rad), dtype=np.int64)
        self.unit = radec_to_unit(self.ra_rad, self.dec_rad)

    @property
    def n(self) -> int:
        return len(self.ra_rad)

    def filter_mag(self, vmag_limit: float) -> "StarCatalog":
        """Keep stars at or brighter than vmag_limit (mirrors Hipparcos.filter)."""
        m = self.vmag <= vmag_limit
        return StarCatalog(self.ra_rad[m], self.dec_rad[m], self.vmag[m],
                           self.ids[m])

    # --- loaders ------------------------------------------------------------
    @classmethod
    def from_arrays(cls, ra_deg, dec_deg, vmag, ids=None) -> "StarCatalog":
        return cls(np.radians(np.asarray(ra_deg, float)),
                   np.radians(np.asarray(dec_deg, float)),
                   np.asarray(vmag, float),
                   None if ids is None else np.asarray(ids))

    @classmethod
    def from_csv(cls, path, ra_col="ra_deg", dec_col="dec_deg",
                 mag_col="vmag", id_col=None) -> "StarCatalog":
        """Load from a CSV with named columns (uses only numpy)."""
        import csv
        ra, dec, mag, ids = [], [], [], []
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                ra.append(float(row[ra_col]))
                dec.append(float(row[dec_col]))
                mag.append(float(row[mag_col]))
                if id_col:
                    ids.append(row[id_col])
        return cls.from_arrays(ra, dec, mag, ids or None)

    @classmethod
    def load_hipparcos_vizier(cls, vmag_limit: float = 6.5) -> "StarCatalog":
        """
        Best-effort real catalog via astroquery (Vizier Hipparcos). Requires
        `pip install astroquery` and network. Raises with a clear message if
        unavailable — the pipeline itself does not depend on this.
        """
        try:
            from astroquery.vizier import Vizier
        except Exception as e:  # pragma: no cover - optional path
            raise RuntimeError(
                "load_hipparcos_vizier needs astroquery (pip install astroquery)"
            ) from e
        v = Vizier(columns=["HIP", "RAICRS", "DEICRS", "Vmag"],
                   column_filters={"Vmag": f"<{vmag_limit}"},
                   row_limit=-1)
        tab = v.get_catalogs("I/311/hip2")[0]  # Hipparcos-2
        return cls.from_arrays(tab["RAICRS"], tab["DEICRS"], tab["Vmag"],
                               tab["HIP"])


# ─────────────────────────────────────────────────────────────────────────────
# Catalog pair table  (the k-vector idea from sonic.Kvector, simplified to a
# sorted inter-star-angle table with binary-search range queries)
# ─────────────────────────────────────────────────────────────────────────────
class _PairTable:
    """
    All catalog star pairs within `max_fov` of each other, stored as directed
    pairs sorted by inter-star angle for O(log n) range queries. This is the
    practical equivalent of SONIC's k-vector: it turns "which catalog pairs are
    ~theta apart?" into a binary search.
    """

    def __init__(self, unit: np.ndarray, max_fov_rad: float):
        n = unit.shape[1]
        cos_max = math.cos(max_fov_rad)
        a_list, b_list, ang_list = [], [], []
        # O(n^2) build, thresholded by FOV. Fine for magnitude-limited catalogs
        # (a few thousand stars); use a spatial index for very large catalogs.
        for i in range(n):
            dots = unit[:, i] @ unit[:, i + 1:]
            js = np.nonzero(dots >= cos_max)[0] + (i + 1)
            if js.size:
                angs = np.arccos(np.clip(dots[js - (i + 1)], -1.0, 1.0))
                a_list.append(np.full(js.size, i))
                b_list.append(js)
                ang_list.append(angs)
        if a_list:
            a = np.concatenate(a_list)
            b = np.concatenate(b_list)
            ang = np.concatenate(ang_list)
        else:
            a = b = np.empty(0, int)
            ang = np.empty(0)
        # Directed (both orderings) so either endpoint can be the shared vertex.
        d_a = np.concatenate((a, b))
        d_b = np.concatenate((b, a))
        d_ang = np.concatenate((ang, ang))
        order = np.argsort(d_ang, kind="stable")
        self.a = d_a[order]
        self.b = d_b[order]
        self.ang = d_ang[order]

    def query(self, ang: float, tol: float):
        """Return (I_array, J_array): directed catalog pairs with angle≈`ang`."""
        lo = np.searchsorted(self.ang, ang - tol, side="left")
        hi = np.searchsorted(self.ang, ang + tol, side="right")
        return self.a[lo:hi], self.b[lo:hi]


# ─────────────────────────────────────────────────────────────────────────────
# Star identification  (mirror sonic.StarId.interstarAngle / checkTriad)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SolveResult:
    ra_deg: float
    dec_deg: float
    roll_deg: float
    n_matches: int
    dcm: np.ndarray                       # world->camera, u_cam ≈ dcm @ u_world
    matches: np.ndarray                   # 2xK: [measurement idx; catalog idx]
    fov_deg: float
    residual_arcsec: float


def _boresight_radec_roll(T: np.ndarray) -> tuple[float, float, float]:
    """world->camera DCM -> (ra_deg, dec_deg, roll_deg) of the camera +z axis."""
    b = T[2, :]                           # boresight (+z) expressed in world
    ra, dec = unit_to_radec(b)
    ra_deg, dec_deg = math.degrees(ra[0]), math.degrees(dec[0])
    # Roll: position angle of camera "up" (-y) from celestial north toward east.
    b = b / np.linalg.norm(b)
    pole = np.array([0.0, 0.0, 1.0])
    north = pole - (pole @ b) * b
    if np.linalg.norm(north) < 1e-9:      # boresight at a celestial pole
        return ra_deg, dec_deg, 0.0
    north /= np.linalg.norm(north)
    east = np.cross(pole, b)
    east /= np.linalg.norm(east)
    cam_up = -T[1, :]
    cam_up = cam_up - (cam_up @ b) * b
    roll = math.degrees(math.atan2(cam_up @ east, cam_up @ north))
    return ra_deg, dec_deg, roll


def identify(los: np.ndarray, catalog: StarCatalog, camera: CameraModel,
             tol_rad: Optional[float] = None, max_fov_rad: Optional[float] = None,
             min_matches: int = 5, confirm_frac: float = 0.4,
             max_triad_seeds: int = 15,
             pair_table: Optional[_PairTable] = None):
    """
    Lost-in-space identification from camera-frame line-of-sight vectors.

    Faithful to sonic.StarId.interstarAngle: search star triads, match their
    inter-star angles to catalog pairs via the pair table, solve Wahba for a
    provisional attitude, then grow/verify inliers; the first triad passing the
    acceptance test wins and the attitude is recomputed from all inliers.

    Acceptance requires BOTH an absolute floor (`min_matches`, >=4 like SONIC)
    AND a confirmation fraction (`confirm_frac` of the measured stars). The
    fraction gate rejects false triads: a correct attitude explains nearly all
    stars in view, whereas a spurious triad explains only its 3 seed stars plus
    a couple of coincidences. Set `confirm_frac=0` for exact SONIC behaviour.

    Returns (matches 2xK, dcm 3x3) or (None, None) on failure.
    """
    if min_matches < 4:
        raise ValueError("min_matches must be >= 4 (SONIC requires >=4)")
    los = _normalize_cols(np.asarray(los, float))
    m = los.shape[1]
    if m < min_matches:
        return None, None

    if max_fov_rad is None:
        max_fov_rad = camera.diag_fov_rad
    if tol_rad is None:
        # Default tolerance ~2 px of centroid error, in angle.
        tol_rad = 2.0 * camera.ifov_rad
    if pair_table is None:
        pair_table = _PairTable(catalog.unit, max_fov_rad)

    cat_u = catalog.unit
    cos_tol = math.cos(tol_rad)
    cos_max = math.cos(max_fov_rad)

    # Pre-query directed candidate pairs per measured pair, on demand + cached.
    ang_cache: dict[tuple[int, int], float] = {}

    def meas_angle(i, j):
        key = (i, j) if i < j else (j, i)
        a = ang_cache.get(key)
        if a is None:
            a = math.acos(float(np.clip(los[:, i] @ los[:, j], -1.0, 1.0)))
            ang_cache[key] = a
        return a

    min_confirm = max(min_matches, int(math.ceil(confirm_frac * m)))
    seeds = min(m, max_triad_seeds)
    # SONIC-style spread-out triad ordering (try well-separated indices first).
    for dj in range(1, seeds - 1):
        for dk in range(1, seeds - dj):
            for i in range(0, seeds - dj - dk):
                j, k = i + dj, i + dj + dk
                if k >= m:
                    continue
                res = _check_triad(los, cat_u, (i, j, k), pair_table, tol_rad,
                                   cos_tol, cos_max, meas_angle, min_confirm)
                if res is not None:
                    return res
    return None, None


def _check_triad(los, cat_u, ijk, pt, tol, cos_tol, cos_max, meas_angle,
                 min_matches):
    i, j, k = ijk
    ang_ij = meas_angle(i, j)
    ang_ik = meas_angle(i, k)
    ang_jk = meas_angle(j, k)

    Iij, Jij = pt.query(ang_ij, tol)          # directed pairs ~ang_ij
    Iik, Kik = pt.query(ang_ik, tol)
    Ijk, Kjk = pt.query(ang_jk, tol)
    if Iij.size == 0 or Iik.size == 0 or Ijk.size == 0:
        return None

    # For shared vertex i: map i->I, j->J (from ij) and i->I, k->K (from ik),
    # then require (J,K) to be an ~ang_jk pair (jk set), mirroring checkTriad.
    from collections import defaultdict
    ik_by_I = defaultdict(list)
    for I2, K in zip(Iik, Kik):
        ik_by_I[I2].append(K)
    jk_set = set(zip(Ijk.tolist(), Kjk.tolist()))

    triangles = []
    for I, J in zip(Iij, Jij):
        for K in ik_by_I.get(I, ()):
            if (J, K) in jk_set:
                triangles.append((I, J, K))
    if not triangles:
        return None

    u_cam = los[:, [i, j, k]]
    center = u_cam.mean(axis=1)
    center /= np.linalg.norm(center)

    for (I, J, K) in triangles:
        u_world = cat_u[:, [I, J, K]]
        try:
            T = wahba_svd(u_cam, u_world)
        except ValueError:
            continue

        # Reproject all measurements into world; keep catalog near triad center.
        center_world = T.T @ center
        near = (center_world @ cat_u) >= cos_max
        cat_near = cat_u[:, near]
        near_idx = np.nonzero(near)[0]
        if cat_near.shape[1] == 0:
            continue

        meas_world = T.T @ los
        cosang = meas_world.T @ cat_near              # M x Nnear
        hit = cosang >= cos_tol
        meas_i, cat_j = np.nonzero(hit)
        if meas_i.size == 0:
            continue

        # Prune double assignments (keep first), like SONIC's quick unique prune.
        _, keep = np.unique(meas_i, return_index=True)
        meas_i, cat_j = meas_i[keep], cat_j[keep]
        _, keep2 = np.unique(cat_j, return_index=True)
        meas_i, cat_j = meas_i[keep2], cat_j[keep2]

        if meas_i.size >= min_matches:
            matches = np.vstack((meas_i, near_idx[cat_j]))
            # Recompute attitude from ALL inliers.
            T = wahba_svd(los[:, matches[0]], cat_u[:, matches[1]])
            return matches, T
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Centroiding  (mirror sonic.Centroider.COB — center of brightness)
# ─────────────────────────────────────────────────────────────────────────────
def estimate_threshold(img: np.ndarray, n_sigma: float = 5.0) -> float:
    """Robust background threshold: median + n_sigma * (MAD-based sigma)."""
    a = np.asarray(img, float)
    med = np.median(a)
    mad = np.median(np.abs(a - med))
    sigma = 1.4826 * mad if mad > 0 else a.std()
    return float(med + n_sigma * sigma)


def centroid(img: np.ndarray, thresh: Optional[float] = None,
             min_size: int = 4, max_size: int = 200, nms_dist: float = 5.0,
             buffer: int = 0):
    """
    Center-of-brightness star detection (port of Centroider.COB).

    Returns (centroids 2xK [x;y] pixels, brightness K,) sorted brightest-first.
    Needs opencv OR scipy for connected components; raises a clear error if
    neither is installed (the solver core does not require this function).
    """
    a = np.asarray(img)
    if a.ndim == 3:
        a = a.mean(axis=2)
    a = a.astype(float)
    n, mcols = a.shape
    if thresh is None:
        thresh = estimate_threshold(a)

    bw = a > thresh
    labels, num = _connected_components(bw)
    if num == 0:
        return np.empty((2, 0)), np.empty(0)

    cx, cy, bright = [], [], []
    for lbl in range(1, num + 1):
        idx = np.nonzero(labels.ravel() == lbl)[0]
        if idx.size < min_size or idx.size > max_size:
            continue
        rows, cols = np.divmod(idx, mcols)
        if (rows.min() < buffer or rows.max() >= n - buffer or
                cols.min() < buffer or cols.max() >= mcols - buffer):
            continue
        dn = a.ravel()[idx]
        s = dn.sum()
        cx.append((cols * dn).sum() / s)
        cy.append((rows * dn).sum() / s)
        bright.append(s)

    if not cx:
        return np.empty((2, 0)), np.empty(0)
    cx, cy, bright = np.array(cx), np.array(cy), np.array(bright)

    # Non-max suppression: brightest wins within nms_dist.
    order = np.argsort(-bright)
    keep = []
    for o in order:
        if all(math.hypot(cx[o] - cx[k], cy[o] - cy[k]) >= nms_dist for k in keep):
            keep.append(o)
    keep = np.array(keep)
    return np.vstack((cx[keep], cy[keep])), bright[keep]


def _connected_components(bw: np.ndarray):
    if _HAVE_CV2:
        num, labels = cv2.connectedComponents(bw.astype(np.uint8), connectivity=8)
        return labels, num - 1  # cv2 label 0 is background
    if _HAVE_SCIPY:
        labels, num = _ndimage.label(bw, structure=np.ones((3, 3)))
        return labels, num
    raise RuntimeError(
        "centroid() needs opencv (cv2) or scipy for connected components; "
        "install one, or pass line-of-sight vectors to solve()/identify() "
        "directly.")


# ─────────────────────────────────────────────────────────────────────────────
# Top-level solve
# ─────────────────────────────────────────────────────────────────────────────
def solve(source, camera: CameraModel, catalog: StarCatalog,
          vmag_limit: Optional[float] = None, tol_rad: Optional[float] = None,
          min_matches: int = 5, confirm_frac: float = 0.4,
          max_stars: int = 20, **centroid_kw) -> Optional[SolveResult]:
    """
    Solve a frame end to end. `source` may be:
      - a 2-D image (ndarray)  -> centroided here (needs opencv/scipy), or
      - a 2xN array of pixel centroids [x;y], or
      - a 3xN array of camera-frame line-of-sight unit vectors.

    Returns a SolveResult, or None if identification fails.
    """
    cat = catalog.filter_mag(vmag_limit) if vmag_limit is not None else catalog

    src = np.asarray(source, float)
    if src.ndim == 2 and src.shape[0] not in (2, 3):
        # An image: centroid it, brightest first.
        cent, _bright = centroid(src, **centroid_kw)
        los = camera.pixels_to_los(cent[:, :max_stars])
    elif src.shape[0] == 2:
        los = camera.pixels_to_los(src[:, :max_stars])
    elif src.shape[0] == 3:
        los = _normalize_cols(src[:, :max_stars])
    else:
        raise ValueError("source must be an image, 2xN pixels, or 3xN LOS")

    matches, T = identify(los, cat, camera, tol_rad=tol_rad,
                          min_matches=min_matches, confirm_frac=confirm_frac)
    if matches is None:
        return None

    ra_deg, dec_deg, roll_deg = _boresight_radec_roll(T)

    # Residual: mean angle between matched measured LOS and catalog directions.
    mc = los[:, matches[0]]
    mw = cat.unit[:, matches[1]]
    reproj = T @ mw
    dots = np.clip(np.sum(mc * reproj, axis=0), -1.0, 1.0)
    resid_arcsec = math.degrees(float(np.arccos(dots).mean())) * 3600.0

    return SolveResult(ra_deg=ra_deg, dec_deg=dec_deg, roll_deg=roll_deg,
                       n_matches=int(matches.shape[1]), dcm=T, matches=matches,
                       fov_deg=math.degrees(camera.diag_fov_rad),
                       residual_arcsec=resid_arcsec)


# ─────────────────────────────────────────────────────────────────────────────
# Synthetic starfield  (mirror sonic.Camera.synthImage) — for testing w/o data
# ─────────────────────────────────────────────────────────────────────────────
def random_dcm(rng: np.random.Generator) -> np.ndarray:
    """Uniform-ish random rotation via QR of a Gaussian matrix."""
    Q, R = np.linalg.qr(rng.standard_normal((3, 3)))
    Q = Q @ np.diag(np.sign(np.diag(R)))
    if np.linalg.det(Q) < 0:
        Q[:, 0] = -Q[:, 0]
    return Q


def synth_view(camera: CameraModel, catalog: StarCatalog, T: np.ndarray,
               pixel_noise: float = 0.0, rng: Optional[np.random.Generator] = None):
    """
    Project catalog stars into the camera for attitude T (world->camera).
    Returns (pixels 2xK, catalog_idx K) for stars landing on the sensor.
    """
    rng = rng or np.random.default_rng(0)
    u_cam = T @ catalog.unit
    pix, in_front = camera.los_to_pixels(u_cam)
    h = camera.res[0] if camera.res else 2 * camera.v_p
    w = camera.res[1] if camera.res else 2 * camera.u_p
    on = in_front & (pix[0] >= 0) & (pix[0] < w) & (pix[1] >= 0) & (pix[1] < h)
    idx = np.nonzero(on)[0]
    p = pix[:, idx]
    if pixel_noise > 0 and p.shape[1]:
        p = p + rng.normal(0, pixel_noise, p.shape)
    return p, idx


# ─────────────────────────────────────────────────────────────────────────────
# Self-test: end-to-end synthetic solve (numpy only)
# ─────────────────────────────────────────────────────────────────────────────
def _selftest(trials: int = 20, seed: int = 12345) -> int:
    rng = np.random.default_rng(seed)
    # A random dense synthetic catalog so a narrow-ish FOV still holds enough
    # stars — this validates the port without needing Hipparcos data.
    n_stars = 4000
    v = _normalize_cols(rng.standard_normal((3, n_stars)))
    ra, dec = unit_to_radec(v)
    mag = rng.uniform(1.0, 6.0, n_stars)
    cat = StarCatalog(ra, dec, mag)

    cam = dwarf3_wide_camera()
    pair_table = _PairTable(cat.unit, cam.diag_fov_rad)   # build once, reuse

    print(f"cam: hfov={math.degrees(cam.hfov_rad):.1f}°  "
          f"diag={math.degrees(cam.diag_fov_rad):.1f}°  "
          f"ifov={math.degrees(cam.ifov_rad)*3600:.1f}\"/px  "
          f"catalog={cat.n} stars, opencv={_HAVE_CV2} scipy={_HAVE_SCIPY}")

    ok = 0
    max_err = 0.0
    for t in range(trials):
        T_true = random_dcm(rng)
        pix, idx = synth_view(cam, cat, T_true, pixel_noise=0.3, rng=rng)
        if pix.shape[1] < 6:
            continue  # too few stars in view this pointing; skip
        los = cam.pixels_to_los(pix)
        matches, T_est = identify(los, cat, cam, min_matches=5,
                                  pair_table=pair_table)
        if matches is None:
            print(f"  trial {t:2d}: FAIL (no solution, {pix.shape[1]} stars in view)")
            continue
        err = math.degrees(attitude_angle_between(T_true, T_est)) * 3600.0
        ra0, dec0, roll = _boresight_radec_roll(T_est)
        ra_t, dec_t, _ = _boresight_radec_roll(T_true)
        max_err = max(max_err, err)
        good = err < 60.0  # arcsec
        ok += good
        print(f"  trial {t:2d}: {pix.shape[1]:2d} stars, {matches.shape[1]:2d} "
              f"matched  att_err={err:6.1f}\"  "
              f"RA={ra0:7.2f}/{ra_t:7.2f}  Dec={dec0:+6.2f}/{dec_t:+6.2f}  "
              f"{'OK' if good else 'BAD'}")

    print(f"\n{ok}/{trials} solves within 60\"; worst attitude error "
          f"{max_err:.1f}\".")
    return 0 if ok >= max(1, trials - 2) else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
