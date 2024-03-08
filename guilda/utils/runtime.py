import sys
import io
from contextlib import redirect_stdout

def suppress_stdout(func):
    def wrapper(*args, **kwargs):
        with io.StringIO() as buf, redirect_stdout(buf):
            return func(*args, **kwargs)
    return wrapper

def del_cache(obj, prop):
    if prop in obj.__dict__:
        delattr(obj, prop)
        return True
    return False