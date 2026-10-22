# BVSLAM.
#
# Build (from the repository root):
#   docker build -f docker/bvslam.Dockerfile -t bvslam .
# Run (GPU optional; the PyPI torch wheels bundle CUDA, the host only needs the NVIDIA driver):
#   docker run --rm --gpus all -v /path/to/data:/data bvslam \
#     bvslam-run --data-dir /data/<sequence> --calibration-dir /data/<calibration> --out /data/out/trajectory.tum
FROM python:3.11-slim

COPY --from=ghcr.io/astral-sh/uv:0.11.11 /uv /uvx /bin/

# git: LightGlue is a git dependency; libgl1/libglib2.0-0: runtime libraries of opencv-python.
RUN apt-get update \
    && apt-get install -y --no-install-recommends git libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

ENV UV_LINK_MODE=copy \
    UV_COMPILE_BYTECODE=1 \
    UV_PYTHON_DOWNLOADS=never \
    TORCH_HOME=/opt/torch

WORKDIR /opt/bvslam/bvslam
# Dependencies first (cached layer), then the project itself.
COPY bvslam/pyproject.toml bvslam/uv.lock bvslam/.python-version bvslam/README.md ./
RUN uv sync --frozen --no-install-project
COPY bvslam/src ./src
COPY bvslam/configs ./configs
COPY bvslam/tests ./tests
RUN uv sync --frozen

# SuperPoint and LightGlue weights are fetched through torch.hub on first use;
# download them now so the container runs offline.
RUN uv run --frozen python -c "from lightglue import SuperPoint; import kornia.feature as KF; SuperPoint(); KF.LightGlueMatcher('superpoint')"

ENV PATH=/opt/bvslam/bvslam/.venv/bin:$PATH
CMD ["bvslam-run", "--help"]
