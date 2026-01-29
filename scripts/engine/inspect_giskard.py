#!/usr/bin/env python3
import inspect
import sys

try:
    from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose, CartesianPosition, CartesianOrientation
    from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
except ImportError as e:
    print(f"Import failed: {e}")
    sys.exit(1)

def print_init_args(cls):
    sig = inspect.signature(cls.__init__)
    print(f"\n{cls.__name__} __init__ arguments:")
    for name, param in sig.parameters.items():
        print(f"  {name}: {param.annotation} = {param.default}")

print_init_args(CartesianPose)
print_init_args(CartesianPosition)
print_init_args(JointPositionList)
