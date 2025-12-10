"""Code to help initialize pytest."""

import sys
from pathlib import Path

# Add the src directory to the path so that the quaternion_ekf package can be imported
my_path = Path(__file__).parent
sys.path.insert(0, str(my_path / "../src"))
