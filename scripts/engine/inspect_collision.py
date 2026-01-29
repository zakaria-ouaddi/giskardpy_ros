#!/usr/bin/env python3
import inspect
import sys

try:
    from giskardpy.model.collision_matrix_manager import CollisionRequest
except ImportError as e:
    print(f"Import failed: {e}")
    sys.exit(1)

sig = inspect.signature(CollisionRequest.__init__)
print(f"CollisionRequest __init__ arguments:")
for name, param in sig.parameters.items():
    print(f"  {name}: {param.annotation} = {param.default}")
