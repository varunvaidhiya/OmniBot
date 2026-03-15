"""
vla_serve.models.base
~~~~~~~~~~~~~~~~~~~~~
Abstract base class for Vision-Language-Action models.

Implement this interface to plug any VLA model into the vla_serve
inference server without changing any server code.

Example::

    from vla_serve.models.base import VLAModel

    class MyModel(VLAModel):
        def load_model(self, model_path, **kwargs):
            ...

        def predict_action(self, image, instruction, **kwargs):
            ...
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Union

import numpy as np
from PIL import Image


class VLAModel(ABC):
    """Abstract interface for Vision-Language-Action models."""

    @abstractmethod
    def load_model(self, model_path: str, **kwargs) -> None:
        """
        Load (or download) the model.

        Args:
            model_path: HuggingFace model ID or local directory path.
            **kwargs:   Implementation-specific options
                        (e.g., load_in_4bit=True for quantization).
        """

    @abstractmethod
    def predict_action(
        self,
        image: Union[Image.Image, np.ndarray],
        instruction: str,
        **kwargs,
    ) -> Union[List[float], Dict[str, Any]]:
        """
        Predict robot action from an image and natural-language instruction.

        Args:
            image:       RGB input as a PIL Image or H×W×3 uint8 numpy array.
            instruction: Natural-language task description.
            **kwargs:    Optional per-call overrides.

        Returns:
            Action vector as a list of floats, or a dict with at minimum
            a 'vector' key containing the action values.
        """
