# Partially inspired by
# https://github.com/ultralytics/ultralytics/blob/main/docker/Dockerfile
# Use `cuda12.4`, same as installed the one on training server
FROM pytorch/pytorch:2.6.0-cuda12.4-cudnn9-runtime

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    MKL_THREADING_LAYER=GNU \
    OMP_NUM_THREADS=1 \
    YOLO_AUTOINSTALL="False"

# Downloads to user config dir
ADD https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.ttf \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/Arial.Unicode.ttf \
    /root/.config/Ultralytics/

# Install linux packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gcc git zip unzip wget curl htop libgl1 libglib2.0-0 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create working directory
WORKDIR /ultralytics

ADD https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt .

# Install deps
RUN python3 -m pip install --upgrade pip uv
COPY requirements.txt ./
RUN uv pip install --system -r requirements.txt
RUN uv pip install ultralytics --system --no-deps

# Copy contents
COPY config/ config/
COPY src/ src/

CMD ["sh", "-c", "python src/main.py"]
