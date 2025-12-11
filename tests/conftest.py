"""Code to help initialize pytest."""

import sys
from pathlib import Path

import pytest

# Add the src directory to the path so that the quaternion_ekf package can be imported
my_path = Path(__file__).parent
sys.path.insert(0, str(my_path / "../src"))


# Test constants
@pytest.fixture
def nonexistent_port():
    """Provide a non-existent port path for testing error handling."""
    return Path("/dev/nonexistent")


@pytest.fixture
def test_port():
    """Provide a test port path for mocking."""
    return Path("/dev/test")


@pytest.fixture
def mock_response():
    """Provide a mock motor response byte string."""
    return b"\xaa\x05\x45\x00\x00\x00\x00\xbb"
