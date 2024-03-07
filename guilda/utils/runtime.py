import sys
import io
from contextlib import redirect_stdout

def suppress_stdout(func):
    def wrapper(*args, **kwargs):
        with io.StringIO() as buf, redirect_stdout(buf):
            return func(*args, **kwargs)
    return wrapper
