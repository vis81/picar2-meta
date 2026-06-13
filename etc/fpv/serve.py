#!/usr/bin/env python3
"""
Tiny stdlib HTTPS static-file server for the Quest WebXR client.

MediaMTX doesn't serve arbitrary static files (only its hardcoded /<path>/
WebRTC viewer pages), so we run this alongside MediaMTX on a different port.
Both reuse the same self-signed cert so the user only accepts it once.

Usage:
    serve.py [--port 8443] [--root PATH] [--cert FILE] [--key FILE]

Env vars are used if flags aren't given:
    FPV_UI_PORT, FPV_UI_ROOT, FPV_UI_CERT, FPV_UI_KEY
"""
from __future__ import annotations

import argparse
import http.server
import os
import ssl
import sys


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("FPV_UI_PORT", "8443")),
    )
    ap.add_argument(
        "--root",
        default=os.environ.get(
            "FPV_UI_ROOT",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "public"),
        ),
    )
    ap.add_argument(
        "--cert",
        default=os.environ.get("FPV_UI_CERT", "/etc/mediamtx/fpv.crt"),
    )
    ap.add_argument(
        "--key",
        default=os.environ.get("FPV_UI_KEY", "/etc/mediamtx/fpv.key"),
    )
    ap.add_argument(
        "--bind",
        default="0.0.0.0",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()

    if not os.path.isdir(args.root):
        print(f"root not found: {args.root}", file=sys.stderr)
        return 1
    for p in (args.cert, args.key):
        if not os.path.isfile(p):
            print(f"missing cert/key file: {p}", file=sys.stderr)
            return 1

    os.chdir(args.root)

    handler = http.server.SimpleHTTPRequestHandler
    server = http.server.ThreadingHTTPServer((args.bind, args.port), handler)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(args.cert, args.key)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    print(f"[fpv-ui] serving {args.root} on https://{args.bind}:{args.port}",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
