# pylint: disable=C0114,C0115,C0116,R0914

import unittest
from app.module.module import Module


class ModuleTest(unittest.TestCase):
    def test_run(self) -> None:
        module: Module = Module()
        module.run()
