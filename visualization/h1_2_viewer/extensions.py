"""Output-only extension boundary for future navigation and control-panel status.

No policy loader, command publishers or service clients belong in this module.
Navigation geometry uses an explicit world frame; sensor alignment requires validated TF.
"""
import numpy as np
import rerun as rr


class DashboardExtensions:
    def __init__(self, world_frame='map'):
        self.world_frame = world_frame

    def _points(self, points, frame_id):
        xyz = np.asarray(points, dtype=float)
        if frame_id != self.world_frame:
            raise ValueError(f'Expected {self.world_frame} frame, got {frame_id}')
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not np.all(np.isfinite(xyz)):
            raise ValueError('Expected finite Nx3 world-frame coordinates')
        return xyz

    def log_map(self, points, *, frame_id):
        xyz = self._points(points, frame_id)
        rr.log('navigation', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log('navigation/map', rr.CoordinateFrame(frame=self.world_frame), static=True)
        rr.log('navigation/map', rr.Points3D(xyz, radii=0.02))

    def log_path(self, points, *, frame_id):
        xyz = self._points(points, frame_id)
        rr.log('navigation', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log('navigation/path', rr.CoordinateFrame(frame=self.world_frame), static=True)
        rr.log('navigation/path', rr.LineStrips3D([xyz],
                                               colors=[60, 190, 255]))

    def log_control_status(self, text):
        # Display status only. This is not a robot control channel.
        rr.log('panels/control', rr.TextDocument(str(text)))
