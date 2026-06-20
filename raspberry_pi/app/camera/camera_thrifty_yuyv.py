"""The Thrifty Cam camera, using the YUYV encoding.
"""

# pylint: disable=E0401

from pprint import pprint
from typing import Any

import numpy as np
from numpy.typing import NDArray
from picamera2 import CompletedRequest, Picamera2, libcamera  # type: ignore
from typing_extensions import override

from app.camera.camera_protocol import Request
from app.camera.real_camera import RealCamera
from app.camera.distortion import Distortion
from app.camera.intrinsic import Intrinsic
from app.camera.model import Model
from app.camera.real_request import RealRequest
from app.camera.shutter import Shutter
from app.camera.size import Size
from app.config.identity import Identity
from app.util.timer import Timer

class CameraThriftyYuyv(RealCamera):
    def __init__(self, identity: Identity) -> None:
        print("\n*** Camera: CameraThriftyYuyv")
        Picamera2.set_logging(Picamera2.INFO)
        # debug logs with every frame (!)
        # Picamera2.set_logging(Picamera2.DEBUG)
        print("GLOBAL CAMERA INFO")
        pprint(Picamera2.global_camera_info())
        print("+==================")
        self._cam: Picamera2 = Picamera2()  # type: ignore

        print("\n*** SENSOR MODES AVAILABLE")
        pprint(self._cam.sensor_modes)  # type:ignore

        print("\n*** CAMERA CONTROLS")
        pprint(self._cam.camera_controls)  # type:ignore

        print("\n*** RAW MODES")
        pprint(self._cam._raw_modes)

        self._mtx: Intrinsic = Intrinsic(identity)
        self._dist: Distortion = Distortion(identity)

        model: Model = Model.get(self._cam.camera_properties)  # type: ignore
        self._rolling = Shutter(model).rolling()
        self._size: Size = Size.from_model(model)

        print("\n\n*** CONFIG! ***\n\n")
        self._camera_config: dict[str, Any] = CameraThriftyYuyv.__get_config(  # type: ignore
            identity, self._cam, self._size  # type: ignore
        )

        print("\n*** REQUESTED CONFIG")
        print(self._camera_config)

        # optimal alignment makes the ISP a little faster
        self._cam.align_configuration(self._camera_config, optimal=True)  # type:ignore
        print("\n*** ALIGNED CONFIG")
        print(self._camera_config)

        self._cam.configure(self._camera_config)  # type:ignore

        # the "sensor" is not filled in for USB cameras.
        if identity in (
            Identity.FLIPPED,
            # Identity.FUNNEL,
            Identity.CLIMB_RIGHT,
        ):
            self._fail_mismatched_size()

        self._cam.start()  # type:ignore
        self._frame_time = Timer.time_ns()

    @override
    def capture_request(self) -> Request:
        capture_start: int = Timer.time_ns()
        req: CompletedRequest = self._cam.capture_request()  # type:ignore
        total_time_ms = (capture_start - self._frame_time) / 1000000
        self._frame_time = capture_start
        fps = 1000 / total_time_ms
        return RealRequest(req, fps, self._rolling)  # type: ignore

    @override
    def stop(self) -> None:
        self._cam.stop()  # type: ignore
        print("Camera stop")

    @override
    def get_size(self) -> Size:
        return self._size

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        return self._mtx.get()

    @override
    def get_dist(self) -> NDArray[np.float32]:
        return self._dist.get()

    @staticmethod
    def __get_config(
        identity: Identity, cam: Picamera2, size: Size  # type: ignore
    ) -> dict[str, Any]:
        print("creating config")
    
        camera_config: dict[str, Any] = cam.create_still_configuration(  # type:ignore
            # more buffers seem to make the pipeline a little smoother
            buffer_count=5,
            queue=True,
            main={"format": "YUYV", "size": (size.width, size.height)},
            raw=None,
            transform=RealCamera._transform(identity),  # type:ignore
            controls={
                # ANALOGUE GAIN
                # To minimize blur, set this as high as possible.
                # TODO: try much larger values, up to 250.
                # "AnalogueGain": 8,
                #
                # AUTO EXPOSURE
                # Must be true for outside or in bright sun.
                # "AeEnable": True,
                # "AeEnable": False,
                #
                # AUTO WHITE BALANCE
                # Screws up color sensing.
                # "AwbEnable": False,
                #
                # EXPOSURE TIME (microseconds)
                # Minimizes blur.  Requires pretty good light.
                # "ExposureTime": 500,
                # Works in less light, slightly more blur.
                # "ExposureTime": 2000,
                #
                # COLOUR GAINS
                # The first argument is the red gain, second is blue gain.
                # values are from testing in the new gym lighting(1.2,2.2)
                # "ColourGains": (1.2,2.0),
                #
                # FRAME DURATION LIMITS
                # limit auto exposure: go as fast as possible but no slower than 30fps
                # without a duration limit, we slow down in the dark, which is fine
                # "FrameDurationLimits": (500, 33333),  # 41 fps
                #
                # NOISE REDUCTION MODE
                # noise reduction takes A LOT of time (100 ms per frame!), don't need it.
                # See libcamera.controls.draft.NoiseReductionModeEnum.Off,
                # "NoiseReductionMode": 0,
            },
        )
        return camera_config  # type: ignore

    def _fail_mismatched_size(self):
        """Configured size and actual size must match."""
        if (
            self._camera_config["sensor"]["output_size"]
            != self._cam.camera_config["sensor"]["output_size"]  # type:ignore
        ):
            raise ValueError(
                "Desired sensor size must match selected sensor size.",
                self._camera_config["sensor"]["output_size"],
                self._cam.camera_config["sensor"]["output_size"],  # type:ignore
            )
