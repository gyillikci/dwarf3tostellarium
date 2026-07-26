# OPNAV (SONIC) study — using it in dwarf3tostellarium

A study of the **SONIC** optical-navigation toolkit
([opnavlab/sonic](https://github.com/opnavlab/sonic)) and how its OPNAV
capabilities can be put to work inside `dwarf3tostellarium`.

**SONIC** = **S**oftware for **O**ptical **N**avigation and **I**nstrument
**C**alibration — an object-oriented MATLAB package from the Georgia Tech Space
Exploration Analysis Laboratory ("opnavlab"). It is built on principled
projective geometry and bundles the pieces you need for space-flight optical
navigation. The one capability that matters for a telescope↔Stellarium bridge is
its **lost-in-space star identification + attitude determination** — i.e. a
plate solver: given a star image, recover where the camera is pointed (RA/Dec of
boresight) with no prior guess.

---

## 1. What OPNAV gives us that we don't have today

`dwarf3tostellarium` currently trusts the **DWARF 3's own onboard plate-solve**.
The bridge sends a GOTO (`_do_goto` in `server.py`), the device solves + slews +
tracks, and we report back whatever RA/Dec we commanded. Nothing on our side
ever *independently measures* where the scope actually ended up. The IMU path
(`wit_imu.py`) helps polar alignment, but it is gravity/heading referenced to
~0.01–0.02°, not an absolute celestial fix, and its azimuth drifts in 6-DOF.

OPNAV closes that gap. Feed it a captured frame and it returns an **absolute,
catalog-referenced attitude** — the true boresight RA/Dec and camera roll — from
the stars alone. That unlocks:

- **Independent pointing verification** — did the scope actually land on the
  target the DWARF claimed? Report the *measured* RA/Dec to Stellarium instead of
  the *commanded* one.
- **Closed-loop GOTO correction** — solve the frame, compute the residual
  between measured and target boresight, send a corrective nudge. A "sync"/
  "align" primitive the bridge lacks.
- **Camera-based IMU calibration** — `wit_imu.py --calibrate-*` currently trusts
  the DWARF's tracking + an ephemeris. A star solve gives a true optical
  boresight to calibrate the IMU mount offset against, at night, without needing
  the Sun/Moon.
- **Offline / vendor-independent solving** — if a future firmware breaks the
  onboard solver (the whole reason this repo exists), we can still solve frames
  ourselves.

---

## 2. The SONIC OPNAV pipeline (image → RA/Dec)

The full star-tracker pipeline is assembled from a handful of `+sonic` classes.
Traced end to end:

```
 JPEG frame
    │  sonic.Image + sonic.Centroider.COB(img, thresh)         [Centroider.m]
    ▼
 star centroids (pixels)  → sonic.Points2
    │  camera.Kinv * pixels, then dist_model.undistort(...)    [Camera.m / Pinhole / BrownConrady]
    ▼
 line-of-sight unit vectors in CAMERA frame → sonic.PointsS2
    │  sonic.StarId.interstarAngle(kvec, cat, los, tol, maxFOV, minMatches)   [StarId.m]
    ▼
 (matches to Hipparcos, att_ICRF2C)  ← Wahba solved by SVD    [Attitude.solveWahbasProblem]
    ▼
 boresight RA/Dec = f(att_ICRF2C)  → report to Stellarium
```

### The classes that matter

| SONIC class | Role in the pipeline |
|---|---|
| `sonic.Image` | Wraps the pixel matrix; `estNoiseRand`/`estNoiseSort` give a centroiding threshold. |
| `sonic.Centroider.COB` | Center-of-brightness star detection. Threshold → binarize → connected components → intensity-weighted centroid, with min/max cluster size, non-max suppression and edge buffer. Returns `Points2` centroids. |
| `sonic.Camera` | Camera intrinsics `K` (and analytic `Kinv`), FOV/IFOV, and a distortion model. Built from FOV+resolution (`constructFromFOV`) or from `K` (`constructFromK`). `Kinv` is what turns pixels into image-plane rays. |
| `sonic.DistortionModel` (`Pinhole`, `BrownConrady`) | `distort`/`undistort`. `Pinhole` is a pass-through; `BrownConrady` models real lens distortion (radial + tangential) and is needed for an accurate solve on a wide lens. |
| `sonic.Hipparcos` (a `StarCatalog`) | The star catalog. `eval(et, r_obs_AU)` applies the 5-parameter standard model (proper motion + parallax) to get ICRF unit vectors at your epoch. `filter(...)` trims to a limiting magnitude. |
| `sonic.Kvector` | Precomputed **k-vector** index over catalog inter-star angles for O(1)-ish range queries — the speed trick that makes lost-in-space tractable. |
| `sonic.StarId.interstarAngle` | The identification core. Smart triad search over measured line-of-sight vectors, matched against catalog pairs via the k-vector; validates a triad, then grows/prunes matches. Returns `matches` + `att_ICRF2C`. |
| `sonic.Attitude` | Rotation container (DCM/quat/axis-angle/rotation-vec). `solveWahbasProblem` solves Wahba's problem via **SVD** (Markley's method) from ≥2 matched direction pairs. |

### How identification actually works (`StarId.interstarAngle`)

1. Measured line-of-sight vectors (camera frame) come in as `PointsS2`.
2. It walks **triads** of measured stars in a cache-friendly order (from *"Fast
   and robust kernel generators for star trackers"*, Acta Astronautica 2017).
3. For each triad it computes the three inter-star angles and uses the
   **k-vector** to pull all catalog pairs within `tol` of each angle, then
   intersects the candidate pairs to find self-consistent catalog triangles.
4. A candidate triangle yields a provisional attitude (Wahba/SVD); it reprojects
   all measurements into the catalog, keeps stars within `max_angle` of the triad
   center, and counts inliers within `tol`.
5. First triad reaching `min_matches` (≥4, default 5) wins; it recomputes the
   attitude from **all** inliers and returns `att_ICRF2C`.

`tol` is the angular match tolerance, `max_angle` is bounded by the camera FOV,
and the catalog passed in **must be the full, unfiltered** catalog (the code
enforces `isempty(cat.filter_map)`).

### From attitude to what Stellarium wants

`att_ICRF2C` is the rotation ICRF→camera. The camera **boresight** is the camera
+z axis expressed in ICRF, i.e. the 3rd row of the DCM:
`b_ICRF = att.dcm(3,:).'`. Convert that unit vector to RA/Dec
(`RA = atan2(b_y, b_x)`, `Dec = asin(b_z)`) and you have exactly the J2000 RA/Dec
that `server.py`'s `_build_position_packet` already encodes for Stellarium. The
in-plane camera roll comes from the other two DCM rows — useful for overlaying a
solved frame or de-rotating for stacking.

---

## 3. The integration problem: SONIC is MATLAB, this repo is Python

This is the central obstacle. SONIC requires:

- **MATLAB** with the **Image Processing** and **Computer Vision** toolboxes
  (per its README, v24.1).
- Separately-downloaded `.mat` data files (`hipparcos.mat`,
  `constellations.mat`) that live in `+sonic/+data/` — **they are not in the
  repo**, only a `readme.md` placeholder is (confirmed: the Box link in the SONIC
  README is the only source). Star ID cannot run without them.

`dwarf3tostellarium` is deliberately dependency-light Python (flask, requests,
websocket-client; opencv/numpy/Pillow/bleak for the GUI). Shipping a MATLAB
runtime to run a plate solve on a hobby telescope bridge is a non-starter for
most users. So there are three realistic strategies:

### Option A — Port the star-ID pipeline to Python  *(recommended)*

The algorithm surface we actually need is small and well-scoped, and every
building block has a mature Python equivalent:

| SONIC piece | Python equivalent |
|---|---|
| `Image` + `Centroider.COB` | `opencv` threshold + `cv2.connectedComponentsWithStats`, or `photutils`/`sep` for weighted centroids (already opencv-adjacent — the GUI ships opencv). |
| `Camera.K/Kinv` + distortion | a 3×3 intrinsics matrix + `cv2.undistortPoints` (Brown–Conrady is exactly OpenCV's distortion model). |
| `Hipparcos.eval` (5-param model) | `astropy`/`skyfield` (skyfield is *already* an optional dep here) applied to a Hipparcos/Tycho subset, or a prebuilt bright-star table. |
| `Kvector` + `StarId.interstarAngle` | port the triad/k-vector matcher (~a few hundred lines) — or reuse a ready star tracker like **`tetra3`** (ESA/observation-friendly, pure Python) which implements the same lost-in-space idea. |
| `Attitude.solveWahbasProblem` | Wahba via `numpy.linalg.svd` — a ~5-line function, identical to SONIC's `U*diag([1,1,det(U)det(V)])*V'`. |

SONIC then serves as the **authoritative algorithm spec and validation oracle**:
implement the Python path, and cross-check its centroids, line-of-sight vectors,
and recovered attitude against SONIC run on the same frame in MATLAB. This keeps
the runtime pure-Python and optional.

### Option B — Call MATLAB from Python (MATLAB Engine API)

`matlab.engine` lets Python drive SONIC directly. **Accurate but heavy**:
requires a licensed MATLAB + toolboxes on the host and the `.mat` catalogs.
Reasonable for a bench/dev workflow where the maintainer has MATLAB and wants to
prototype against the real SONIC; unreasonable as a shipped feature. Good as a
*reference harness* alongside Option A, not as the product path.

### Option C — Use an existing Python plate-solver, OPNAV-informed

`astrometry.net` (local index) or `tetra3` solve frames in pure Python today.
SONIC's value here is as the **design reference** — it tells us exactly which
knobs matter (FOV, limiting magnitude, `tol`, `max_angle`, min matches,
distortion) and gives a trusted oracle to validate against. Least new code;
loses the "we implemented OPNAV ourselves" property.

**Recommendation:** Option A for the shipped feature (pure-Python, optional
import, mirrors SONIC's math), with a small Option-B harness kept out of the
runtime for validation. This matches the repo's existing pattern of optional
heavy deps (skyfield) that degrade gracefully.

---

## 4. Making the camera model fit the DWARF 3

Star ID needs a correct `Camera`. The DWARF 3 has **two lenses**, and the choice
drives everything:

- **Tele (`ch0`)** — long focal length, **very narrow FOV** (the repo uses
  `TELE_OVERLAY_MAG = WIDE_FOV / TELE_FOV ≈ 17.6×`; see `TRACKING_FINDINGS.md`
  §"Tele overlay"). A narrow FOV may contain **too few catalog stars** to hit
  `min_matches` for a lost-in-space solve, but is superb for a *refinement* solve
  once you have a rough attitude.
- **Wide (`ch1`)** — the ~17× wider FOV is the right lens for **lost-in-space**:
  more stars per frame → reliable triad matching. `max_angle` for
  `interstarAngle` should be set to (a bit under) the wide diagonal FOV.

Concrete steps:

1. **Pin the intrinsics.** Build `sonic.Camera` (or the Python equivalent) from
   published tele/wide FOV + sensor resolution via the FOV constructor, then
   **refine by calibration** — SONIC + the frames themselves can recover `K` and
   Brown–Conrady coefficients; the wide lens especially needs real distortion
   terms, not a pinhole, for an accurate solve to the edges.
2. **Set the limiting magnitude** to match what the lens/exposure records, and
   `filter` the catalog to it (fewer, brighter stars = faster, cleaner match).
3. **Choose `tol`** from the pixel centroid accuracy × IFOV (`Camera.ifov_*`).

The tele/wide boresights differ by a fixed offset (`BORESIGHT_DX/DY` in
`roi_gui.py`), so a solve on one lens maps to the other by a known, calibratable
rotation — the same offset the GUI already tunes for overlays.

---

## 5. Where it plugs into this codebase

- **Capture already exists.** `roi_gui.py`'s **📷 Photo (Tele)** button triggers
  `CMD_CAMERA_TELE_PHOTOGRAPH` and logs a timestamped line to `captures.jsonl`.
  The JPEG lands on the device's storage. A solve step consumes that JPEG. (For
  wide-lens lost-in-space we'd add a wide still, or pull a frame from the wide
  RTSP stream.)
- **A new `platesolve.py` module** (pure Python, Option A): `solve(image) ->
  {ra_deg, dec_deg, roll_deg, n_matches, fov_deg}` — centroid → LOS → identify →
  attitude → boresight RA/Dec. Optional import; absent deps degrade gracefully,
  exactly like `skyfield`/`bleak` do today.
- **`captures.jsonl` gains a `solve` block** next to the existing `imu` block, so
  every photo records both the IMU attitude *and* the optical fix — directly
  comparable, which is precisely what IMU mount-offset calibration wants.
- **`server.py` reports measured RA/Dec.** After a solve, feed the boresight into
  the position loop (`_send_position_loop` / `_build_position_packet`) so
  Stellarium's reticle shows where the scope *is*, not where it was *told* to go;
  and expose a `/api/solve` + `/api/sync` endpoint for closed-loop correction.
- **`wit_imu.py` gains a `--calibrate-stars`** mode: solve a night frame, use the
  optical boresight (not the Sun/Moon ephemeris) as truth for the IMU offset.

Note the **single-client control channel** (`TRACKING_FINDINGS.md` §6:
`DEVICE_OCCUPIED`) — solving works on already-captured frames, so it doesn't
contend with the app for the WebSocket. And per the **hardware caution** in that
doc, none of this blind-probes the device; it only consumes frames the existing,
capture-verified photo path produces.

### GMS-Feature-Matcher — the *other* repo in this workspace

`GMS-Feature-Matcher` (Grid-based Motion Statistics, CVPR'17) is
frame-to-frame **feature correspondence**, not star ID. It's the complementary,
*relative* half of OPNAV: match ORB/SIFT features between two frames (e.g.
successive tele frames, or a frame vs. the `sky_reference/` Sun/Moon plates) to
recover **relative** motion/registration — useful for stacking, drift tracking,
and Sun/Moon feature alignment. SONIC handles the *absolute* fix (stars → RA/Dec);
GMS handles *relative* fixes between images. They layer cleanly: absolute solve
to anchor, GMS to propagate cheaply between solves.

---

## 6. Concrete next steps

1. **Reference run (Option B, dev-only):** get the SONIC `.mat` catalogs, run
   `SyntheticStarImgTutorial` + the star-ID path in MATLAB on a real DWARF wide
   frame to produce a trusted attitude → the validation oracle.
2. **Calibrate the wide camera:** recover `K` + Brown–Conrady coefficients for
   the wide lens (and tele) from real frames. This is the single biggest
   accuracy lever.
3. **Prototype `platesolve.py` (Option A) — DONE (see the module in this repo).**
   Pure-Python port of the SONIC pipeline: `CameraModel` (K/Kinv + Brown-Conrady),
   `Centroider.COB`-equivalent center-of-brightness detection, a sorted
   inter-star-angle pair table (the k-vector idea), the `StarId.interstarAngle`
   triad matcher, and `wahba_svd` (Markley's SVD, identical to SONIC). numpy is
   the only hard dep; opencv/scipy are optional and only for centroiding a real
   image. A false-triad **confirmation gate** (`confirm_frac`) was added on top
   of SONIC's `min_matches` floor after the synthetic test surfaced spurious
   locks in dense fields. `python3 platesolve.py` runs a self-contained synthetic
   end-to-end test (no Hipparcos data / no real frame): 20/20 solves, worst
   attitude error 17″. Still TODO: validate against real DWARF frames + SONIC
   (step 1) and wire in a real catalog (`StarCatalog.load_hipparcos_vizier`).
4. **Wire results into `captures.jsonl`, `server.py` (`/api/solve`, `/api/sync`),
   and `wit_imu.py` (`--calibrate-stars`).**
5. **Layer GMS** for relative registration/stacking between absolute solves.

---

## Appendix — SONIC classes seen while studying (`+sonic/`)

Star-tracker / OPNAV core used above: `StarId`, `StarCatalog` + `Hipparcos`,
`Kvector`, `Camera`, `Pinhole`/`BrownConrady`/`DistortionModel`, `Centroider`,
`Image`, `Attitude`, `PointsS2`/`Points2`/`Points3`, `Project`, `Aberration`,
`SphereCoords`, `Units`, `USNOGNC`.

Broader OPNAV toolkit (not needed for plate-solving, but present): horizon-based
navigation and conics (`Conic`, `Quadric`, `Ellipsoid`, `EllipseFitter`,
`EdgeFinder`, `ScanLines`, `Robbins`, `Lines2/3`, `Planes3`, `GeometryP2/P3`,
`MeetJoinable`), pose/position (`PnP`, `Pose`, `PositionEstimation`, `Lambert`),
rendering + reflectance (`OrthoRender`, `OrthoSphere`, `OrthoPatch`,
`Reflectance`, `Hapke`, `LommelSeeliger`, `Lambert`, `LunarLambert`, `OrenNayar`,
`Chandrasekhar`), and support (`Math`, `Constants`, `Tolerances`, `SampleGeom2D`,
`SphereCoords`).
