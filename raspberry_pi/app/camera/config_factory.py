# pylint: disable=R0903

from app.camera.config_protocol import Config
from app.config.identity import Identity
from app.camera.config_thrifty_yuyv import ConfigThriftyYuyv
from app.camera.config_thrifty_mjpeg import ConfigThriftyMjpeg
from app.camera.config_gs_color import ConfigGsColor
from app.camera.config_gs_mono import ConfigGsMono


class ConfigFactory:
    """Choose a camera confing based on identity."""

    @staticmethod
    def get(identity: Identity) -> Config:
        match identity:
            case Identity.FUNNEL:
                return ConfigThriftyYuyv()
            case Identity.CLIMB_RIGHT:
                return ConfigThriftyMjpeg()
            case Identity.CLIMB_LEFT:
                return ConfigGsColor()
            case Identity.SHOOTER:
                return ConfigGsMono()
            case _:
                return ConfigGsMono()
