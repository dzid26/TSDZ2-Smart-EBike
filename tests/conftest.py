import subprocess
from tests.load_c_code import load_code

def pytest_sessionstart(session):
    """
    Called after the Session object has been created and
    before performing collection and entering the run test loop.
    """
    load_code('_tsdz2')


def pytest_configure(config):
    """
    This hook is called for every plugin and initial conftest
    file after command line options have been parsed.
    """

def pytest_sessionfinish(session, exitstatus):
    """
    Called after whole test run finished, right before
    returning the exit status to the system.
    """

def pytest_unconfigure(config):
    """
    called before test process is exited.
    """
