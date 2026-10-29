"""Compatibility entry point. The implementation lives in the independent visualization package."""

def main():
    try:
        from h1_2_viewer.cli import main as viewer_main
    except ImportError as exc:
        raise SystemExit('Install the standalone viewer first: python -m pip install -e /path/to/humanoid_bridge/visualization') from exc
    viewer_main()


if __name__ == '__main__':
    main()
