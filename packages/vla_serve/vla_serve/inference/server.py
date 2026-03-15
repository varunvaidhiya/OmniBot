"""
vla_serve FastAPI inference server.

Environment variables
---------------------
VLA_MODEL_CLASS   Module path to model class (default: vla_serve.models.openvla.OpenVLAModel)
VLA_MODEL_PATH    HuggingFace model ID or local path (default: openvla/openvla-7b)
VLA_LOAD_4BIT     '1' to enable 4-bit quantization (default: '0')
VLA_AUTO_LOAD     '1' to load model on startup (default: '0' — use /load_model)
"""

import importlib
import os
import time
from contextlib import asynccontextmanager
from typing import Optional

import uvicorn
from fastapi import FastAPI, HTTPException

from .schema import InferenceRequest, InferenceResponse
from ..utils.image import decode_base64_image
from ..models.base import VLAModel

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

_model: Optional[VLAModel] = None


def _get_model_class() -> type:
    cls_path = os.getenv(
        'VLA_MODEL_CLASS', 'vla_serve.models.openvla.OpenVLAModel')
    module_path, cls_name = cls_path.rsplit('.', 1)
    module = importlib.import_module(module_path)
    return getattr(module, cls_name)


# ---------------------------------------------------------------------------
# Lifespan
# ---------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI):
    global _model
    cls = _get_model_class()
    _model = cls()
    print(f'vla_serve: model class = {cls.__name__}')

    if os.getenv('VLA_AUTO_LOAD', '0') == '1':
        model_path = os.getenv('VLA_MODEL_PATH', 'openvla/openvla-7b')
        load_4bit = os.getenv('VLA_LOAD_4BIT', '0') == '1'
        print(f'Auto-loading {model_path} (4bit={load_4bit})...')
        _model.load_model(model_path=model_path, load_in_4bit=load_4bit)

    yield

    if _model is not None:
        del _model


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

app = FastAPI(
    title='vla_serve',
    description='Model-agnostic VLA inference REST server',
    version='1.0.0',
    lifespan=lifespan,
)


@app.get('/health')
def health():
    loaded = (_model is not None and
              getattr(_model, 'model', None) is not None)
    return {'status': 'ok', 'model_loaded': loaded}


@app.post('/load_model')
def load_model(
    model_path: str = 'openvla/openvla-7b',
    load_4bit: bool = False,
):
    if _model is None:
        raise HTTPException(status_code=503, detail='Server not initialized.')
    try:
        _model.load_model(model_path=model_path, load_in_4bit=load_4bit)
        return {'status': 'success', 'model': model_path}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post('/predict', response_model=InferenceResponse)
def predict(request: InferenceRequest):
    if _model is None or getattr(_model, 'model', None) is None:
        raise HTTPException(
            status_code=503,
            detail='Model not loaded. POST to /load_model first.')
    try:
        t0 = time.time()
        image = decode_base64_image(request.image_base64)
        result = _model.predict_action(image, request.instruction,
                                       **(request.config or {}))
        latency = (time.time() - t0) * 1000.0
        action = result if isinstance(result, dict) else {'vector': result}
        return InferenceResponse(
            action=action,
            raw_output=str(result),
            latency_ms=latency,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    uvicorn.run(
        'vla_serve.inference.server:app',
        host='0.0.0.0',
        port=int(os.getenv('VLA_PORT', '8000')),
        reload=False,
    )


if __name__ == '__main__':
    main()
