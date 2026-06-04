#!/usr/bin/env python3
"""
Serves the robot_web_app web app on port 8080.
Launched automatically by teleop.launch.xml.
"""
import os
import re
import csv
import json
import http.server
import yaml
from ament_index_python.packages import get_package_share_directory

webapp_dir = os.path.join(
    get_package_share_directory('robot_web_app'),
    'webapp'
)

_field_pkg = get_package_share_directory('robot_field')
_FIELD_CONFIG_INSTALLED  = os.path.join(_field_pkg, 'config', 'field_config.yaml')
_MISSION_ZONES_INSTALLED = os.path.join(_field_pkg, 'config', 'mission_zones.yaml')

_robot_desc_pkg = get_package_share_directory('robot_description')
MOBILE_BASE_XACRO = os.path.join(_robot_desc_pkg, 'urdf', 'mobile_base.xacro')

_missions_pkg = get_package_share_directory('robot_missions')
# Prefer source-tree files so manual edits and UI saves stay in sync.
# Installed layout: install/<pkg>/share/<pkg> → 4 parents up = workspace root.
_ws_root = _missions_pkg
for _ in range(4):
    _ws_root = os.path.dirname(_ws_root)
_seeds_src = os.path.join(_ws_root, 'src', 'robot_missions', 'config', 'seeds.csv')
SEEDS_CSV_PATH = _seeds_src if os.path.isfile(_seeds_src) else os.path.join(_missions_pkg, 'config', 'seeds.csv')
_SEEDS_INSTALLED_PATH = os.path.join(_missions_pkg, 'config', 'seeds.csv')

_field_config_src  = os.path.join(_ws_root, 'src', 'robot_field', 'config', 'field_config.yaml')
_mission_zones_src = os.path.join(_ws_root, 'src', 'robot_field', 'config', 'mission_zones.yaml')
FIELD_CONFIG_PATH  = _field_config_src  if os.path.isfile(_field_config_src)  else _FIELD_CONFIG_INSTALLED
MISSION_ZONES_PATH = _mission_zones_src if os.path.isfile(_mission_zones_src) else _MISSION_ZONES_INSTALLED

_SEED_FIELDS = ['name', 'spacing_x_mm', 'spacing_y_mm', 'depth_mm', 'tool_id', 'tool_param']
_SEED_INT_FIELDS = {'spacing_x_mm', 'spacing_y_mm', 'depth_mm', 'tool_id', 'tool_param'}


def _load_seeds():
    seeds = []
    try:
        with open(SEEDS_CSV_PATH, newline='') as f:
            # Skip comment and blank lines before passing to DictReader
            data_lines = [l for l in f if l.strip() and not l.lstrip().startswith('#')]
        reader = csv.DictReader(data_lines)
        for row in reader:
            seed = {k: row[k].strip() for k in _SEED_FIELDS}
            for k in _SEED_INT_FIELDS:
                seed[k] = int(seed[k])
            seeds.append(seed)
    except Exception as e:
        print(f'[webapp_server] Warning: could not load seeds: {e}')
    return seeds


def _save_seeds(seeds):
    def _write_csv(path):
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=_SEED_FIELDS)
            writer.writeheader()
            writer.writerows(seeds)
    _write_csv(SEEDS_CSV_PATH)
    if SEEDS_CSV_PATH != _SEEDS_INSTALLED_PATH:
        _write_csv(_SEEDS_INSTALLED_PATH)


def _parse_robot_config():
    """Extract <xacro:property name="..." value="..."/> float values from mobile_base.xacro."""
    props = {}
    try:
        with open(MOBILE_BASE_XACRO) as f:
            content = f.read()
        for m in re.finditer(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"', content):
            try:
                props[m.group(1)] = float(m.group(2))
            except ValueError:
                pass  # skip non-numeric properties (e.g. "${pi/2}")
    except Exception as e:
        print(f'[webapp_server] Warning: could not parse robot config: {e}')
    return props

def _field_changed(old, new):
    """Return True if any field geometry or strip list differs between two field dicts."""
    for key in ('strip_length', 'strip_spacing'):
        if abs(float(old.get(key) or 0) - float(new.get(key) or 0)) > 1e-9:
            return True
    old_o = old.get('origin') or {}
    new_o = new.get('origin') or {}
    for key in ('x', 'y'):
        if abs(float(old_o.get(key) or 0) - float(new_o.get(key) or 0)) > 1e-9:
            return True
    # Yaw tolerance is 1e-4 rad (~0.006°) to absorb degrees-display rounding (2dp → max 8.7e-5 rad error)
    if abs(float(old_o.get('yaw') or 0) - float(new_o.get('yaw') or 0)) > 1e-4:
        return True
    old_s = old.get('strips') or []
    new_s = new.get('strips') or []
    if len(old_s) != len(new_s):
        return True
    for o, n in zip(old_s, new_s):
        if o.get('id') != n.get('id') or bool(o.get('enabled', True)) != bool(n.get('enabled', True)):
            return True
    return False


PORT = 8080


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=webapp_dir, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/api/field_config':
            self._serve_yaml(FIELD_CONFIG_PATH)
        elif self.path == '/api/mission_zones':
            self._serve_yaml(MISSION_ZONES_PATH)
        elif self.path == '/api/robot_config':
            self._serve_json(_parse_robot_config())
        elif self.path == '/api/seeds':
            self._serve_json(_load_seeds())
        else:
            super().do_GET()

    def do_POST(self):
        if self.path == '/api/field_config':
            self._save_field_config()
        elif self.path == '/api/mission_zones':
            self._save_mission_zones()
        elif self.path == '/api/seeds':
            self._save_seeds_handler()
        else:
            self.send_error(404)

    def _save_field_config(self):
        try:
            length = int(self.headers.get('Content-Length', 0))
            incoming = json.loads(self.rfile.read(length).decode('utf-8'))
            # Belt-and-suspenders: reject if any zone is in_progress
            try:
                with open(MISSION_ZONES_PATH, 'r') as f:
                    zones_data = yaml.safe_load(f) or {}
            except Exception:
                zones_data = {}
            if any(z.get('status') == 'in_progress' for z in (zones_data.get('zones') or [])):
                self._serve_json({'ok': False, 'reason': 'in_progress'})
                return
            # Diff strips — wipe planned zones only if something actually changed
            try:
                with open(FIELD_CONFIG_PATH, 'r') as f:
                    current = yaml.safe_load(f) or {}
            except Exception:
                current = {}
            changed = _field_changed(current.get('field') or {}, incoming.get('field') or {})
            wiped = 0
            if changed:
                all_zones = zones_data.get('zones') or []
                kept = [z for z in all_zones if z.get('status') == 'completed']
                wiped = len(all_zones) - len(kept)
                zones_data['zones'] = kept
                for path in {MISSION_ZONES_PATH, _MISSION_ZONES_INSTALLED}:
                    with open(path, 'w') as f:
                        yaml.dump(zones_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            for path in {FIELD_CONFIG_PATH, _FIELD_CONFIG_INSTALLED}:
                with open(path, 'w') as f:
                    yaml.dump(incoming, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            self._serve_json({'ok': True, 'wiped_planned': wiped})
        except Exception as e:
            self.send_error(500, str(e))

    def _save_mission_zones(self):
        try:
            length = int(self.headers.get('Content-Length', 0))
            incoming = json.loads(self.rfile.read(length).decode('utf-8'))
            # Completed zones on disk are immutable — merge them back over whatever the client sent
            try:
                with open(MISSION_ZONES_PATH, 'r') as f:
                    current = yaml.safe_load(f) or {}
            except Exception:
                current = {}
            completed = {z['zone_id']: z for z in (current.get('zones') or []) if z.get('status') == 'completed'}
            merged = []
            seen = set()
            for z in (incoming.get('zones') or []):
                zid = z.get('zone_id')
                merged.append(completed[zid] if zid in completed else z)
                seen.add(zid)
            # Re-add completed zones the client dropped entirely
            for zid, z in completed.items():
                if zid not in seen:
                    merged.append(z)
            incoming['zones'] = merged
            for path in {MISSION_ZONES_PATH, _MISSION_ZONES_INSTALLED}:
                with open(path, 'w') as f:
                    yaml.dump(incoming, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            self._serve_json({'ok': True})
        except Exception as e:
            self.send_error(500, str(e))

    def _save_seeds_handler(self):
        try:
            length = int(self.headers.get('Content-Length', 0))
            seeds = json.loads(self.rfile.read(length).decode('utf-8'))
            _save_seeds(seeds)
            self._serve_json({'ok': True})
        except Exception as e:
            self.send_error(500, str(e))

    def _serve_json(self, data):
        body = json.dumps(data).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_yaml(self, path):
        try:
            with open(path, 'r') as f:
                self._serve_json(yaml.safe_load(f))
        except Exception as e:
            self.send_error(500, str(e))

    def _save_yaml(self, path, also_write=None):
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            data = json.loads(body.decode('utf-8'))
            with open(path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            if also_write and also_write != path:
                with open(also_write, 'w') as f:
                    yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            resp = json.dumps({'ok': True}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)
        except Exception as e:
            self.send_error(500, str(e))


def main():
    server = http.server.HTTPServer(('0.0.0.0', PORT), Handler)
    print(f'[webapp_server] Serving at http://0.0.0.0:{PORT}')
    print(f'[webapp_server] Webapp directory: {webapp_dir}')
    server.serve_forever()


if __name__ == '__main__':
    main()
