"""Reuse this workspace's running server, or reserve an available local port."""
import argparse
import json
import os
from pathlib import Path
import socket
import threading
import time
import urllib.error
import urllib.request
import webbrowser

HERE = Path(__file__).resolve().parent
APP_ID = 'eoreumi-v23'


def is_our_server(port):
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(f'http://127.0.0.1:{port}/health', timeout=.3) as response:
            data = json.loads(response.read(4096))
        return (data.get('app') == APP_ID and
                os.path.normcase(data.get('workspace', '')) == os.path.normcase(str(HERE)))
    except (OSError, ValueError, AttributeError):
        return False


def find_endpoint(first=8230, count=20):
    reserved = None
    for port in range(first, min(first + count, 65536)):
        if is_our_server(port):
            if reserved:
                reserved.close()
            return port, None
        if reserved is None:
            candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                candidate.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            try:
                candidate.bind(('127.0.0.1', port))
            except OSError:
                candidate.close()
            else:
                reserved = candidate
    if reserved is None:
        raise OSError('No free local port. Choose another --port.')
    return reserved.getsockname()[1], reserved


def open_when_ready(port):
    for _ in range(300):
        if is_our_server(port):
            webbrowser.open(f'http://127.0.0.1:{port}/')
            return
        time.sleep(.1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8230)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    port, listener = find_endpoint(args.port)
    url = f'http://127.0.0.1:{port}/'
    if listener is None:
        print('v23 is already running: ' + url, flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        return
    print('Starting v23: ' + url, flush=True)
    try:
        from server import run
        if not args.no_browser:
            threading.Thread(target=open_when_ready, args=(port,), daemon=True).start()
        run(port=port, sock=listener)
    finally:
        listener.close()


if __name__ == '__main__':
    main()
