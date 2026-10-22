"""Raw image loading and Bayer demosaicing straight to grayscale.

The pipeline is grayscale end-to-end: the raw Bayer mosaic is converted to a
single-channel uint8 image with OpenCV's BayerXX2GRAY, which is what the
feature extractor sees.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

# Sensor Bayer pattern -> OpenCV demosaic-to-grayscale code. OpenCV names the
# pattern by the second row/column, so RGGB maps to COLOR_BayerBG2GRAY.
_BAYER_CODES = {
    "RGGB": cv2.COLOR_BayerBG2GRAY,
    "BGGR": cv2.COLOR_BayerRG2GRAY,
    "GRBG": cv2.COLOR_BayerGB2GRAY,
    "GBRG": cv2.COLOR_BayerGR2GRAY,
}


def load_raw(path: str | Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return image


def demosaic(raw: np.ndarray, bayer_pattern: str = "RGGB") -> np.ndarray:
    """Demosaic a single-channel Bayer image to grayscale.

    `bayer_pattern="none"` accepts already-demosaiced input: returned as-is if
    single-channel, converted from BGR or BGRA otherwise.
    """
    pattern = bayer_pattern.upper()
    if pattern == "NONE":
        if raw.ndim == 2:
            return raw
        return cv2.cvtColor(raw, cv2.COLOR_BGRA2GRAY if raw.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
    if pattern not in _BAYER_CODES:
        raise ValueError(f"Unknown bayer_pattern: {bayer_pattern}")
    return cv2.cvtColor(raw, _BAYER_CODES[pattern])


def load_gray(path: str | Path, bayer_pattern: str = "RGGB") -> np.ndarray:
    """Load + demosaic to a single-channel grayscale image."""
    return demosaic(load_raw(path), bayer_pattern)
