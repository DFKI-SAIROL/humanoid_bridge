"""Standalone H1-2 viewer. --demo runs without ROS or the bridge workspace."""
import argparse
from pathlib import Path
import shutil

from .configuration import load_profile


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, help='YAML profile; resource paths are relative to this file')
    parser.add_argument('--urdf', type=Path, help='Override model only, relative to current directory')
    parser.add_argument('--source', choices=['unitree', 'joint_states', 'jsonl'])
    parser.add_argument('--input', default='-', help='JSONL path or - for stdin when --source jsonl')
    parser.add_argument('--state-topic')
    parser.add_argument('--lidar-topic')
    parser.add_argument('--hand-topic')
    parser.add_argument('--language', choices=['zh', 'en'], help='Dashboard labels: Chinese or English')
    parser.add_argument('--full-detail', action='store_true', help='Use original meshes instead of lightweight display meshes')
    parser.add_argument('--hz', type=float)
    parser.add_argument('--stale-after', type=float)
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument('--connect', help='Remote Rerun gRPC URL')
    transport.add_argument('--serve', action='store_true', help='Headless gRPC + browser viewer server')
    parser.add_argument('--observe-client', action='store_true', help='Read-only subscriptions to /robot_cmd and /inspire/cmd')
    parser.add_argument('--grpc-port', type=int, default=9876)
    parser.add_argument('--web-port', type=int, default=9090)
    parser.add_argument('--save', type=Path, help='Record .rrd instead of opening a window')
    parser.add_argument('--blueprint', type=Path, help='Use your saved .rbl layout instead of the default')
    parser.add_argument('--demo', action='store_true', help='Synthetic source; no ROS initialization/imports')
    parser.add_argument('--demo-frames', type=int, default=420)
    parser.add_argument('--check', action='store_true', help='Validate config/model/resources and exit; no viewer or ROS')
    args, remaining = parser.parse_known_args(argv)
    if remaining and remaining[0] != '--ros-args':
        parser.error(f'Unknown arguments: {remaining}; ROS flags must follow --ros-args')
    for port in (args.grpc_port, args.web_port):
        if not 1 <= port <= 65535:
            parser.error('Ports must be in 1..65535')
    if args.grpc_port == args.web_port:
        parser.error('Web and gRPC ports must be different')
    if args.save and (args.serve or args.connect):
        parser.error('--save cannot be combined with --serve/--connect')
    try:
        profile = load_profile(args.config, args.urdf)
        if args.language:
            profile.settings['layout']['language'] = args.language
        if args.full_detail:
            profile.settings['model']['mesh_face_limit'] = 0
        source = profile.settings['source']
        if args.source:
            source['type'] = args.source
            if args.source == 'joint_states':
                source['state_topic'] = '/joint_states'
                source['hand_topic'] = None
        for key in ('state_topic', 'lidar_topic', 'hand_topic'):
            if getattr(args, key) is not None:
                source[key] = getattr(args, key)
        for key in ('hz', 'stale_after'):
            if getattr(args, key) is not None:
                value = getattr(args, key)
                import math
                if not math.isfinite(value) or value <= 0 or (key == 'hz' and value > 120):
                    raise ValueError(f'Invalid --{key.replace("_", "-")}')
                profile.settings['runtime'][key] = value
        if args.demo_frames <= 0:
            raise ValueError('--demo-frames must be positive')
        if args.observe_client:
            source['body_target_topic'] = '/robot_cmd'
            source['hand_target_topic'] = '/inspire/cmd'
        from .model import RobotModel
        model = RobotModel(profile)
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    if model.mesh_stats[0]:
        print(f'Display mesh: {model.mesh_stats[0]:,} → {model.mesh_stats[1]:,} triangles (source model unchanged)', flush=True)
    if args.check:
        print(f'Configuration OK; URDF={model.urdf}; mapped joints={len(profile.joints)}; mesh references={len(model.meshes)}')
        model.close()
        return
    import rerun as rr
    from .layout import blueprint
    from .renderer import Dashboard
    rr.init('h1_2_dashboard_demo' if args.demo else 'h1_2_dashboard')
    layout = blueprint(profile, demo=args.demo)
    try:
        if args.save and (args.serve or args.connect):
            raise ValueError('--save cannot be combined with --serve/--connect')
        if args.serve:
            url = rr.serve_grpc(grpc_port=args.grpc_port, default_blueprint=layout, server_memory_limit=profile.settings['runtime']['server_memory_limit'], newest_first=not args.demo)
            rr.serve_web_viewer(web_port=args.web_port, open_browser=False, connect_to=url)
            from .web import viewer_url
            print(f'\n可视化地址：{viewer_url(args.web_port, args.grpc_port)}\n只读观察；按 Ctrl+C 停止可视化，不影响控制 Client。\n', flush=True)
        elif args.connect:
            rr.connect_grpc(args.connect)
            rr.send_blueprint(layout)
        elif args.save:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            rr.save(args.save, default_blueprint=layout)
        else:
            executable = shutil.which('rerun')
            if not executable:
                raise RuntimeError('rerun executable is missing from the active Python environment')
            rr.spawn(executable_path=executable, memory_limit='512MiB', server_memory_limit='512MiB')
            rr.send_blueprint(layout)
        if args.blueprint:
            # .rbl is deliberately separate from robot geometry and data adapters.
            rr.log_file_from_path(args.blueprint)
        dashboard = Dashboard(model, profile, demo=args.demo)
        if args.demo:
            from .demo import run
            run(dashboard, args.demo_frames)
        elif source['type'] == 'jsonl':
            from .jsonl_source import run
            run(dashboard, args.input)
        else:
            try:
                from .ros2_source import run
            except ImportError as exc:
                raise RuntimeError('Live mode requires ROS 2 in the current Python environment. Use --demo for offline preview.') from exc
            run(dashboard, profile, remaining)
        if args.serve:
            import time
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        rr.disconnect()
        model.close()


if __name__ == '__main__':
    main()
