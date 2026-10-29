"""Guard display-only optimization and unchanged-sample suppression."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from h1_2_viewer.mesh_lod import simplify_stl, _STL
from h1_2_viewer.renderer import Dashboard
from h1_2_viewer.i18n import translator


class DisplayOptimization(unittest.TestCase):
    def test_mesh_simplification_preserves_source_and_bounds(self):
        with tempfile.TemporaryDirectory() as d:
            import struct
            root = Path(d)
            # Closed bipyramid: simplification must preserve nonempty finite geometry.
            points = np.array([[np.cos(t), np.sin(t), 0] for t in np.linspace(0, 2*np.pi, 64, endpoint=False)] + [[0,0,1],[0,0,-1]])
            faces = np.array([[i, (i+1)%64, 64] for i in range(64)] + [[(i+1)%64,i,65] for i in range(64)])
            data = np.zeros(len(faces), dtype=_STL); data['vertices'] = points[faces]
            raw = b'QA'.ljust(80,b'\0') + struct.pack('<I',len(faces)) + data.tobytes()
            source, output = root/'model.stl', root/'display.stl'; source.write_bytes(raw)
            before, after = simplify_stl(source,output,32)
            self.assertEqual(before,128); self.assertLessEqual(after,32); self.assertGreater(after,0)
            self.assertEqual(source.read_bytes(),raw)
            actual=np.frombuffer(output.read_bytes(),dtype=_STL,offset=84)['vertices']
            self.assertTrue(np.all(np.isfinite(actual)))
            self.assertLess(np.max(np.abs(actual)),1.1)

    def test_unchanged_images_are_not_sent_again(self):
        dashboard=Dashboard.__new__(Dashboard); dashboard._images={}
        pixels=np.zeros((4,4,3),dtype=np.uint8)
        with patch('h1_2_viewer.renderer.rr.log') as log:
            dashboard.image('rgb',pixels); dashboard.image('rgb',pixels.copy())
            self.assertEqual(log.call_count,1)
            pixels[0,0]=255; dashboard.image('rgb',pixels)
            self.assertEqual(log.call_count,2)

    def test_language_switch(self):
        self.assertEqual(translator({'layout':{'language':'zh'}})('Observe'),'主界面')
        self.assertEqual(translator({'layout':{'language':'en'}})('Observe'),'Observe')
