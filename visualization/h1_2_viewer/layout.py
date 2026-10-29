"""Compact observation workspace; diagnostics live on a separate page."""
import rerun.blueprint as rrb


def blueprint(profile, demo=False):
    layout = profile.settings['layout']
    groups = profile.groups
    from .i18n import translator
    tr = translator(profile.settings)

    def metric_tabs(metric, unit):
        return rrb.Tabs(*[
            rrb.TimeSeriesView(origin='/',
                contents=[f'/joints/{metric}/{group}/**', f'/commands/body/{group}/**'] if metric == 'position' else [f'/joints/{metric}/{group}/**'],
                name=f'{tr(group)} · {unit}')
            for group in groups], name=tr(metric),
            active_tab=list(groups).index(layout['default_joint_group'])
            if layout['default_joint_group'] in groups else 0)

    scene = rrb.Spatial3DView(
        origin='/scene' if profile.settings['attitude']['enabled'] and not demo else '/robot',
        contents=['/robot/**', '/navigation/**'], name=tr('H1-2 · Scene'),
        background=layout['background'], line_grid=False,
        eye_controls=rrb.EyeControls3D(position=layout['model_eye'],
                                     look_target=layout['model_target'], eye_up=[0, 0, 1]))
    observation = rrb.Vertical(
        scene,
        rrb.Horizontal(
            rrb.Spatial2DView(origin='/cameras/head/rgb', name=tr('Head · RGB')),
            rrb.Spatial2DView(origin='/cameras/left/rgb', name=tr('Left hand · RGB')),
            rrb.Spatial2DView(origin='/cameras/right/rgb', name=tr('Right hand · RGB')),
            column_shares=[1, 1, 1]),
        row_shares=layout['row_shares'], name=tr('Observe'))
    diagnostics = rrb.Tabs(
        rrb.TextDocumentView(origin='/panels/status', name=tr('Connection')),
        rrb.Tabs(metric_tabs('position', 'rad'), metric_tabs('velocity', 'rad/s'),
                 metric_tabs('torque', 'Nm'),
                 rrb.Tabs(rrb.TimeSeriesView(origin='/hands', name=tr('Hand feedback')),
                          rrb.TimeSeriesView(origin='/commands/hands', name=tr('Hand targets')), name=tr('Hands')),
                 rrb.TextDocumentView(origin='/panels/joints', name=tr('Joint values')), name=tr('Joints')),
        rrb.TextDocumentView(origin='/panels/spatial', name=tr('Frames')),
        rrb.Vertical(
            rrb.Horizontal(
                rrb.TimeSeriesView(origin='/imu/roll_pitch', name=tr('Roll / Pitch · deg')),
                rrb.TimeSeriesView(origin='/imu/yaw', name=tr('Yaw · deg')),
                column_shares=[1, 1]),
            rrb.TimeSeriesView(origin='/imu/quaternion', name=tr('Quaternion · xyzw')),
            rrb.TextDocumentView(origin='/panels/attitude', name=tr('IMU status')),
            row_shares=[.45, .35, .20], name='IMU'),
        rrb.TextDocumentView(origin='/panels/sensors', name=tr('Sensors')),
        rrb.Spatial2DView(origin='/cameras/head/depth', name=tr('Head · Depth')),
        name=tr('Debug'), active_tab=0)
    return rrb.Blueprint(
        rrb.Tabs(observation, diagnostics, active_tab=0, name=tr('H1-2')),
        rrb.TimePanel(expanded=False, timeline='elapsed',
                      play_state=rrb.components.PlayState.Playing if demo else rrb.components.PlayState.Following,
                      loop_mode=rrb.components.LoopMode.All if demo else rrb.components.LoopMode.Off),
        rrb.BlueprintPanel(expanded=True), rrb.SelectionPanel(expanded=False),
        auto_layout=False, auto_views=False)
