"""Browser URLs for a locally hosted viewer; no machine-specific configuration."""
from urllib.parse import urlencode


def viewer_url(web_port, grpc_port):
    return f'http://localhost:{web_port}/?' + urlencode({
        'url': f'rerun+http://localhost:{grpc_port}/proxy'})
