FROM nvcr.io/nvidia/pytorch:25.06-py3

RUN apt update
RUN apt upgrade -y
RUN apt install -y ffmpeg libsm6 libxext6
RUN apt clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt ./
RUN pip install -r requirements.txt
RUN pip install ultralytics --no-deps
RUN python3 -c "import torch, tensorrt; print('Torch:', torch.__version__, 'TRT:', tensorrt.__version__)"

COPY lib/ lib/
COPY main.py ./

ENV MODEL_PATH=/opt/models/model.engine

EXPOSE 8000
CMD ["sh", "-c", "python main.py --model $MODEL_PATH"]
