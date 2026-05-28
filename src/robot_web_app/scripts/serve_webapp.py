#!/usr/bin/env python3
"""
Serves the robot_web_app web app on port 8080.
Launched automatically by teleop.launch.xml.
"""
import os
import http.server
import threading
from ament_index_python.packages import get_package_share_directory

# Find the webapp directory inside the installed package
webapp_dir = os.path.join(
    get_package_share_directory('robot_web_app'),
    'webapp'
)

PORT = 8080

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=webapp_dir, **kwargs)
    def log_message(self, format, *args):
        pass  # suppress per-request logs

def main():
    server = http.server.HTTPServer(('0.0.0.0', PORT), Handler)
    print(f'[webapp_server] Serving at http://0.0.0.0:{PORT}')
    print(f'[webapp_server] Webapp directory: {webapp_dir}')
    server.serve_forever()

if __name__ == '__main__':
    main()