"""Code to help initialize pytest."""

import os
import sys

# Add the src directory to the path so that the quaternion_ekf package can be imported
my_path = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(my_path, "../src"))
