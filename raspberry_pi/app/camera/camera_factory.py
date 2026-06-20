"""Choose a camera implementation based on identity."""

# pylint: disable=C0415,R0903

from app.camera.camera_protocol import Camera
from app.config.identity import Identity


class CameraFactory:
    @staticmethod
    def get(identity: Identity) -> Camera:
        try:
            # these imports will fail if we're not running on a Raspberry Pi.
            match identity:
                case Identity.FUNNEL:
                    from app.camera.camera_thrifty_yuyv import CameraThriftyYuyv
                    return CameraThriftyYuyv(identity)
                case Identity.CLIMB_RIGHT:
                    from app.camera.camera_thrifty_mjpeg import CameraThriftyMjpeg
                    return CameraThriftyMjpeg(identity)
                case Identity.CLIMB_LEFT:
                    from app.camera.camera_gs_color import CameraGsColor
                    return CameraGsColor(identity)
                case Identity.SHOOTER:
                    from app.camera.camera_gs_mono import CameraGsMono
                    return CameraGsMono(identity)
                case _:
                    from app.camera.fake_camera import FakeCamera
                    return FakeCamera("tag_and_board.jpg", (5504, 3096), -0.1, 0.1)

        except ImportError:
            from app.camera.fake_camera import FakeCamera

            # 1/4 scale
            # return FakeCamera("tag_and_board.jpg", (1100, 620), -5)
            # full-size (huge)
            return FakeCamera("tag_and_board.jpg", (5504, 3096), -0.1, 0.1)
            # return FakeCamera("blob.jpg")
