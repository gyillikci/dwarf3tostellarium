"""
Reimplementation of the two DWARF3 astro image-processing algorithms that were
fully traced from the `magni` firmware via Ghidra (see memory: dwarf3-tracking-
protocol.md, "live-stacking algorithm fully traced").

LiveStacker  -- mirrors the firmware's `overlayimage` function:
    1. cv2.findTransformECC(reference, new_frame, motionType=MOTION_EUCLIDEAN)
       to register each new frame against a fixed reference (the first frame).
    2. cv2.warpAffine to align the new frame onto the reference's pixel grid.
    3. A masked running-mean accumulator: a float64 sum buffer and a per-pixel
       uint16 valid-sample counter, incremented only where the warped frame is
       non-zero, so pixels invalidated by the warp (frame edges) don't get
       diluted by phantom zero samples.

unsharp_mask -- mirrors the firmware's `imgSharpen` function:
    result = src*(1+amount) - GaussianBlur(src)*amount

Firmware used fixed internal constants for the Gaussian kernel/sigma and the
sharpen amount that were never extracted (only the call structure was), so
those are exposed here as tunable parameters instead.
"""

import numpy as np
import cv2


class LiveStacker:
    """Incremental mean-stacker with ECC frame registration.

    Usage:
        stacker = LiveStacker()
        for frame in frames:          # frame: single-channel, uint8 or uint16
            stacker.add_frame(frame)
        result = stacker.get_stack()  # dtype matches the input frames
    """

    def __init__(self, ecc_iterations=50, ecc_eps=1e-6):
        self.motion_type = cv2.MOTION_EUCLIDEAN
        self.criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            ecc_iterations,
            ecc_eps,
        )
        self.reference_gray = None   # fixed float32 grayscale registration target
        self.input_dtype = None
        self.sum_buf = None          # float64 accumulator, same shape as input
        self.count_buf = None        # uint16 per-pixel valid-sample count
        self.frames_added = 0
        self.frames_rejected = 0

    def reset(self):
        self.__init__()

    @staticmethod
    def _to_registration_gray(frame):
        """Scale to a float32 single-channel image suitable for ECC."""
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        # Normalize into a modest float range regardless of input bit depth
        # (the firmware scaled by a fixed constant before ECC; any consistent
        # scale works since ECC only cares about relative gradients).
        maxval = float(np.iinfo(frame.dtype).max) if np.issubdtype(frame.dtype, np.integer) else 1.0
        return (gray.astype(np.float32) / maxval)

    def add_frame(self, frame):
        """Register `frame` against the reference, warp it into alignment, and
        fold it into the running mean. Returns True if the frame was
        successfully aligned and accumulated, False if it was rejected
        (ECC failed to converge -- the firmware silently drops such frames)."""
        if self.reference_gray is None:
            self.input_dtype = frame.dtype
            self.reference_gray = self._to_registration_gray(frame)
            self.sum_buf = frame.astype(np.float64).copy()
            self.count_buf = np.ones(frame.shape[:2], dtype=np.uint16)
            self.frames_added += 1
            return True

        new_gray = self._to_registration_gray(frame)
        warp_matrix = np.eye(2, 3, dtype=np.float32)
        try:
            _, warp_matrix = cv2.findTransformECC(
                self.reference_gray, new_gray, warp_matrix,
                self.motion_type, self.criteria,
            )
        except cv2.error:
            self.frames_rejected += 1
            return False

        h, w = self.reference_gray.shape
        aligned = cv2.warpAffine(
            frame, warp_matrix, (w, h),
            flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
        )

        valid = aligned != 0
        self.sum_buf[valid] += aligned[valid]
        self.count_buf[valid] += 1
        self.frames_added += 1
        return True

    def get_stack(self):
        """Current running-mean stacked image, cast back to the input dtype."""
        if self.sum_buf is None:
            return None
        count = np.maximum(self.count_buf, 1).astype(np.float64)
        if self.sum_buf.ndim == 3:
            count = count[..., None]
        mean = self.sum_buf / count
        info = np.iinfo(self.input_dtype)
        return np.clip(mean, info.min, info.max).astype(self.input_dtype)


def unsharp_mask(image, amount=1.0, sigma=1.0, ksize=(0, 0)):
    """Classic unsharp mask: result = src*(1+amount) - blurred*amount.

    Matches the firmware's `imgSharpen`: GaussianBlur then
    cv2.addWeighted(src, 1+amount, blurred, -amount, 0). `amount` and `sigma`
    are firmware-unknown tunables; 1.0 / 1.0 are reasonable defaults.
    """
    blurred = cv2.GaussianBlur(image, ksize, sigma)
    return cv2.addWeighted(image, 1.0 + amount, blurred, -amount, 0)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("usage: python astro_processing.py <image1> [image2] [image3] ...")
        print("       stacks the given frames and writes stacked_result.png / sharpened_result.png")
        sys.exit(1)

    stacker = LiveStacker()
    for path in sys.argv[1:]:
        frame = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if frame is None:
            print(f"skip (unreadable): {path}")
            continue
        ok = stacker.add_frame(frame)
        print(f"{'ok  ' if ok else 'FAIL'} {path}")

    result = stacker.get_stack()
    if result is not None:
        cv2.imwrite("stacked_result.png", result)
        sharpened = unsharp_mask(result.astype(np.float32), amount=1.0, sigma=1.5)
        cv2.imwrite("sharpened_result.png", np.clip(sharpened, 0, 255).astype(np.uint8))
        print(f"stacked {stacker.frames_added} frame(s), rejected {stacker.frames_rejected}")
        print("wrote stacked_result.png, sharpened_result.png")
