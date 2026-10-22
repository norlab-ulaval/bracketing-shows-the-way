"""Windowed landmark-track bookkeeping.

Maps a (frame_idx, left_keypoint_idx) sighting to a persistent landmark id.
There is no re-identification after a track leaves the lookback window (no
loop closure): a point seen again later becomes a new landmark.
"""

from __future__ import annotations


class LandmarkTracker:
    def __init__(self, first_id: int = 0) -> None:
        self.next_id: int = first_id
        # frame_idx -> {left_keypoint_idx -> landmark_id}
        self._tracks: dict[int, dict[int, int]] = {}

    def get_or_create(self, frame_idx: int, left_kp_idx: int, existing_landmark_id: int | None = None) -> tuple[int, bool]:
        """Look up (or allocate) the landmark id for a keypoint sighting.

        `existing_landmark_id`, if given, is the id already assigned to the same
        physical point in another frame; it is propagated to this sighting.

        Returns (landmark_id, is_new_landmark).
        """
        frame_tracks = self._tracks.setdefault(frame_idx, {})
        if left_kp_idx in frame_tracks:
            return frame_tracks[left_kp_idx], False

        if existing_landmark_id is not None:
            frame_tracks[left_kp_idx] = existing_landmark_id
            return existing_landmark_id, False

        landmark_id = self.next_id
        self.next_id += 1
        frame_tracks[left_kp_idx] = landmark_id
        return landmark_id, True

    def landmark_id_at(self, frame_idx: int, left_kp_idx: int) -> int | None:
        return self._tracks.get(frame_idx, {}).get(left_kp_idx)

    def sightings(self, frame_idx: int) -> dict[int, int]:
        return dict(self._tracks.get(frame_idx, {}))

    def evict_before(self, cutoff_frame_idx: int) -> None:
        for idx in [idx for idx in self._tracks if idx < cutoff_frame_idx]:
            del self._tracks[idx]
