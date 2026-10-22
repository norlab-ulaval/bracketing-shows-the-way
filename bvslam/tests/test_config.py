from pathlib import Path

from bvslam.config import Config

CONFIGS = Path(__file__).resolve().parents[1] / "configs"


def test_paper_yaml_equals_defaults() -> None:
    assert Config.load(CONFIGS / "paper.yaml") == Config()

