# pylint: disable=C0114,C0115,C0116,R0903,R0914

import unittest

loader = unittest.TestLoader()
suite = loader.discover("tests", pattern="*test.py")

runner = unittest.TextTestRunner(verbosity=2)
runner.run(suite)
