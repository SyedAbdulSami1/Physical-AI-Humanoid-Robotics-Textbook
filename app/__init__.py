# This file makes the 'app' directory a Python package.

from . import schemas
# Only import models if it has content
try:
    from . import models
except ImportError:
    # models module may be empty or not exist
    pass