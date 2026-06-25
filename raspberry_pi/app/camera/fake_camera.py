# pylint: disable=E1101,R0903,R1732

from pathlib import Path
from typing import Optional
import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from typing_extensions import override
from app.camera.camera_protocol import Camera
from app.camera.fake_request import FakeRequest
from app.camera.size import Size
from app.util.timer import Timer


class FakeCamera(Camera):
    """A camera for desktop testing."""

    def __init__(
        self,
        filename: str,
        size: Optional[tuple[int, int]] = None,
        k1: float = 0.0,
    ) -> None:
        """
        filename: in this directory.
        size: if no size is supplied, the native size is used.
        k1: quadratic distortion term for undistortion
        """
        p = Path(__file__).parent / filename
        pathstr: str = str(p)
        # force the file to be read as three-channel BGR
        file = cv2.imread(pathstr, cv2.IMREAD_COLOR)
        if file is None:
            raise ValueError("no file: " + pathstr)
        # img is 3 channel BGR
        self.img: MatLike = file
        if size is not None:
            self.img = cv2.resize(self.img, size)
        self.h: int = self.img.shape[0]
        self.w: int = self.img.shape[1]
        self.c: int = self.img.shape[2]
        self.frame_time = Timer.time_ns()
        self.k1 = k1

        # Here we want to *distort* the image, so the the "undistort" below inverts the distortion.
        self.img: MatLike = self.redistort(self.img)

        # uncomment to see the distorted thing
        # cv2.imwrite("debug.jpg", self.img)

    def redistort(self, undistorted_img: MatLike) -> MatLike:
        """Use "remap" to invert the undistortion function."""
        # Create a grid of every pixel coordinate in the target image
        grid_y, grid_x = np.indices((self.h, self.w), dtype=np.float32)
        flat_grid = np.stack([grid_x.ravel(), grid_y.ravel()], axis=-1).reshape(
            -1, 1, 2
        )

        # Use the numerical inverter to find where clean pixels map to
        distort_map = cv2.undistortPoints(
            flat_grid, self.get_intrinsic(), self.get_dist(), P=self.get_intrinsic()
        )
        distort_map = distort_map.reshape(self.h, self.w, 2)

        map_x = distort_map[:, :, 0]
        map_y = distort_map[:, :, 1]

        # Sample from your clean image to generate a synthetically distorted one
        return cv2.remap(undistorted_img, map_x, map_y, interpolation=cv2.INTER_LINEAR)

    @override
    def capture_request(self) -> FakeRequest:
        capture_start: int = Timer.time_ns()
        total_time_ms = (capture_start - self.frame_time) / 1000000
        self.frame_time = capture_start
        fps = 1000 / total_time_ms
        return FakeRequest(self.img, fps)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(
            sensor_width=self.w,
            sensor_height=self.h,
            width=self.w,
            height=self.h,
        )

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        # convenient for experiments related to the GS camera
        return np.array(
            [
                [935, 0, self.w // 2],
                [0, 935, self.h // 2],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> NDArray[np.float32]:
        k1 = self.k1  # radial quadratic term
        k2 = 0  # radial quartic term
        p1 = 0  # tangential
        p2 = 0  # tangential
        return np.array([k1, k2, p1, p2])
