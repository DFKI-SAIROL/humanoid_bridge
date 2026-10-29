"""Compatibility discovery for the independent viewer test suite."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'visualization/tests'))
from test_viewer import CoreTests, ModelIntegrationTests

from test_spatial import SpatialContracts, SpatialModelIntegration
