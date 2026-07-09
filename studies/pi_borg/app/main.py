# pylint: disable=C0114,C0115,C0116,R0903,R0914

from app.module.module import Module

def main() -> None:
    print("main")
    module: Module = Module()
    module.run()
