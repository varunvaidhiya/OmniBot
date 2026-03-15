# vla_serve

> Model-agnostic **Vision-Language-Action** inference REST server.
> Bring any VLA model online with a single command.

## Features

- Clean `VLAModel` ABC — implement two methods to add any model
- Built-in **OpenVLA** implementation (HuggingFace `openvla/openvla-7b`)
- FastAPI REST server with `/health`, `/load_model`, `/predict` endpoints
- 4-bit quantization support via `bitsandbytes`
- Docker image for one-command GPU deployment
- Environment-variable configuration — no code changes needed

## Install

```bash
# Core server only
pip install vla-serve

# With OpenVLA support
pip install "vla-serve[openvla]"
```

## Quick Start

```bash
# Start server (model loaded lazily)
vla-serve

# Load model
curl -X POST "http://localhost:8000/load_model?model_path=openvla/openvla-7b"

# Run inference
curl -X POST http://localhost:8000/predict \
  -H "Content-Type: application/json" \
  -d '{
    "instruction": "Pick up the red cup",
    "image_base64": "<base64-encoded-jpeg>"
  }'
```

## Docker

```bash
# Build and run
docker build -t vla-serve .
docker run --gpus all -p 8000:8000 \
  -e VLA_AUTO_LOAD=1 \
  -e VLA_MODEL_PATH=openvla/openvla-7b \
  vla-serve
```

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `VLA_MODEL_CLASS` | `vla_serve.models.openvla.OpenVLAModel` | Model class path |
| `VLA_MODEL_PATH` | `openvla/openvla-7b` | HuggingFace ID or local path |
| `VLA_LOAD_4BIT` | `0` | Enable 4-bit quantization |
| `VLA_AUTO_LOAD` | `0` | Load model on server startup |
| `VLA_PORT` | `8000` | Server port |

## Add Your Own Model

```python
from vla_serve.models.base import VLAModel
from PIL import Image
import numpy as np

class MyModel(VLAModel):
    def load_model(self, model_path, **kwargs):
        self.model = ...  # load your model here

    def predict_action(self, image, instruction, **kwargs):
        # image is a PIL Image (RGB)
        return {'vector': [0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

```bash
VLA_MODEL_CLASS=mypackage.MyModel vla-serve
```

## API

### `GET /health`
Returns `{"status": "ok", "model_loaded": true/false}`

### `POST /load_model?model_path=...&load_4bit=false`
Loads the model. Returns `{"status": "success"}`.

### `POST /predict`
```json
{
  "instruction": "Pick up the red cup",
  "image_base64": "<base64>",
  "config": {}
}
```
Returns:
```json
{
  "action": {"vector": [0.1, 0.2, ...]},
  "raw_output": "...",
  "latency_ms": 42.3
}
```

## License

Apache-2.0
