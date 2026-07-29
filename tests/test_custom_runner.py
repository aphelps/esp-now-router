# Minimal PlatformIO custom test runner: our host tests own their main() and signal failure via a
# non-zero exit code (the CHECK macro sets it), so we just let the base runner build + run the test
# program and treat the exit code as the verdict. Keeps the tests plain c++ (also runnable via
# `make -C tests test`) with no Unity dependency.
from platformio.test.runners.base import TestRunnerBase


class CustomTestRunner(TestRunnerBase):
    pass
