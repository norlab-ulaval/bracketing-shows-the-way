"""Loading of a (bracketed) stereo sequence and its optional per-frame exposure metadata.

Expected layout of a sequence directory:

    images_left/<timestamp_ns>.<ext>        any image format OpenCV reads (png, jpg, ...)
    images_right/<timestamp_ns>.<ext>       same timestamps as the left images
    images_meta_left/images_meta_left.csv   optional; columns `timestamp` and `ae_zone`

`ae_zone` is the exposure bracket of each frame (0 = shortest, 1 = medium,
2 = longest). Without it, every frame is treated as the same exposure.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from bvslam.imaging import load_gray

# Exposure zone reported by the capture driver -> bracket label.
# Zone 0 is the shortest exposure (SAE), 1 the medium (MAE), 2 the longest (LAE).
_AE_ZONE_SLOT_LABELS = {"0": "SAE", "1": "MAE", "2": "LAE"}
_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".pgm", ".ppm"}


def _images_by_timestamp(images_dir: Path) -> dict[int, Path]:
    return {int(p.stem): p for p in images_dir.iterdir() if p.suffix.lower() in _IMAGE_SUFFIXES and p.stem.isdigit()}


@dataclass
class FrameInfo:
    timestamp_ns: int
    image_path: Path
    slot_label: str  # "SAE" | "MAE" | "LAE" | "UNKNOWN"
    right_image_path: Path | None = None

    @property
    def timestamp_s(self) -> float:
        return self.timestamp_ns * 1e-9


def load_sequence(data_dir: str | Path, side: str = "left") -> list[FrameInfo]:
    """List images_{side}/<timestamp>.<ext>, labelled with images_meta_{side}/images_meta_{side}.csv.

    Returns frames sorted by timestamp. With a metadata file, images without a
    metadata row are skipped; without one (or without an `ae_zone` column),
    every frame gets the medium-exposure label.
    """
    data_dir = Path(data_dir)
    images = _images_by_timestamp(data_dir / f"images_{side}")
    meta_csv = data_dir / f"images_meta_{side}" / f"images_meta_{side}.csv"

    slots: dict[int, str] | None = None
    if meta_csv.exists():
        with open(meta_csv, "r", newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is not None and "ae_zone" in reader.fieldnames:
                slots = {int(row["timestamp"]): _AE_ZONE_SLOT_LABELS.get(row["ae_zone"], "UNKNOWN") for row in reader}

    frames: list[FrameInfo] = []
    for ts, image_path in images.items():
        if slots is None:
            slot = "MAE"
        elif ts in slots:
            slot = slots[ts]
        else:
            continue
        frames.append(FrameInfo(timestamp_ns=ts, image_path=image_path, slot_label=slot))

    frames.sort(key=lambda fr: fr.timestamp_ns)
    return frames


def load_stereo_sequence(data_dir: str | Path) -> list[FrameInfo]:
    """Left frames (see load_sequence) that also have a right image with the same timestamp."""
    data_dir = Path(data_dir)
    right_by_ts = _images_by_timestamp(data_dir / "images_right")

    frames = []
    for fr in load_sequence(data_dir, side="left"):
        right_path = right_by_ts.get(fr.timestamp_ns)
        if right_path is None:
            continue
        fr.right_image_path = right_path
        frames.append(fr)
    return frames


def drop_low_information_frames(
    frames: list[FrameInfo],
    bayer_pattern: str = "RGGB",
    min_brightness: float = 1.0,
    max_brightness: float = 245.0,
) -> list[FrameInfo]:
    """Keep frames whose mean left-image intensity (0-255) lies in [min_brightness, max_brightness]."""
    kept = []
    for fr in frames:
        mean_brightness = float(load_gray(fr.image_path, bayer_pattern).mean())
        if min_brightness <= mean_brightness <= max_brightness:
            kept.append(fr)
    return kept
