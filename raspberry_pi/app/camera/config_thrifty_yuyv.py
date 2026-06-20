# pylint: disable=E0401

from typing import Any
from typing_extensions import override

from app.camera.config_protocol import Config
from app.camera.size import Size


class ConfigThriftyYuyv(Config):
    """The Thrifty Cam camera, using the YUYV encoding."""

    def __init__(self) -> None:
        print("\n*** Config: ConfigThriftyYuyv")

    @override
    def main(self, size: Size) -> dict[str, Any]:
        return {"format": "YUYV", "size": (size.width, size.height)}

    def controls(self) -> dict[str, Any]:
        return {
            # ANALOGUE GAIN
            # To minimize blur, set this as high as possible.
            # On the Thiftycam the range is [1.0, 4.0]
            # "AnalogueGain": 4.0,
            #
            # AUTO EXPOSURE
            # Must be true for outside or in bright sun.
            # "AeEnable": True,
            # "AeEnable": False,
            #
            # EXPOSURE TIME (microseconds)
            # Minimizes blur.  Requires pretty good light.
            # "ExposureTime": 500,
            # Works in less light, slightly more blur.
            # "ExposureTime": 2000,
        }
