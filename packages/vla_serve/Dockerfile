FROM nvidia/cuda:12.1.1-cudnn8-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y \
    python3.11 python3.11-pip python3.11-dev \
    && rm -rf /var/lib/apt/lists/*

RUN ln -sf /usr/bin/python3.11 /usr/bin/python3 && \
    ln -sf /usr/bin/pip3.11 /usr/bin/pip3

WORKDIR /app
COPY . /app

# Install with OpenVLA extras
RUN pip3 install --no-cache-dir ".[openvla]"

# Server config via environment variables
ENV VLA_MODEL_CLASS=vla_serve.models.openvla.OpenVLAModel
ENV VLA_MODEL_PATH=openvla/openvla-7b
ENV VLA_LOAD_4BIT=0
ENV VLA_AUTO_LOAD=0
ENV VLA_PORT=8000

EXPOSE 8000

CMD ["vla-serve"]
