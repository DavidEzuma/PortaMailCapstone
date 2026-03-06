import json
import time
import urllib.request
import urllib.error

CONFIG_PATH = 'external_controller/config.json'


def load_config():
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        return json.load(f)


def http_get_json(url):
    with urllib.request.urlopen(url, timeout=5) as resp:
        if resp.status != 200:
            raise RuntimeError(f'GET {url} failed: {resp.status}')
        data = resp.read().decode('utf-8')
        return json.loads(data)


def http_post_json(url, body_dict):
    data = json.dumps(body_dict).encode('utf-8')
    req = urllib.request.Request(url, data=data, method='POST')
    req.add_header('Content-Type', 'application/json')
    with urllib.request.urlopen(req, timeout=5) as resp:
        if resp.status != 200:
            raise RuntimeError(f'POST {url} failed: {resp.status}')
        raw = resp.read().decode('utf-8')
        return json.loads(raw) if raw else {}


def event_key(evt):
    ts = evt.get('ts')
    name = evt.get('name') or evt.get('type')
    if ts and name:
        return f'{ts}|{name}'
    return None


def post_mode(base_url, mode):
    resp = http_post_json(f'{base_url}/api/mode', {'mode': mode})
    print(f'[EXT] POST /api/mode {mode} -> {resp}')


def main():
    cfg = load_config()
    base_url = cfg.get('lcd_base_url', 'http://127.0.0.1:5050')
    poll_interval = float(cfg.get('poll_interval_sec', 1.0))
    dedup_size = int(cfg.get('dedup_cache_size', 300))

    seen = []
    pending = []
    active = None
    mode = 'IDLE'
    arrival_at = None

    print('[EXT] controller started')

    try:
        while True:
            try:
                events = http_get_json(f'{base_url}/api/events')
            except Exception as exc:
                print(f'[EXT] events poll error: {exc}')
                time.sleep(poll_interval)
                continue

            if isinstance(events, list):
                for evt in events:
                    key = event_key(evt) or ''
                    if not key or key in seen:
                        continue
                    seen.append(key)
                    if len(seen) > dedup_size:
                        seen = seen[-dedup_size:]
                    name = evt.get('name') or evt.get('type')
                    print(f'[EXT] event: {name}')

                    if name == 'start_room1':
                        if 'ROOM1' not in pending:
                            pending.append('ROOM1')
                    elif name == 'start_room2':
                        if 'ROOM2' not in pending:
                            pending.append('ROOM2')
                    elif name == 'delivery_confirmed':
                        if active:
                            print(f'[EXT] Delivery confirmed for {active}, returning to dock')
                            post_mode(base_url, 'DOCK_IDLE')
                            active = None
                            mode = 'RETURNING'

            if not active and pending:
                if 'ROOM1' in pending:
                    active = 'ROOM1'
                else:
                    active = pending[0]
                if active in pending:
                    pending.remove(active)
                mode = 'DELIVERING'
                arrival_at = time.time() + 3.0
                print(f'[EXT] Starting delivery for {active}')

            if active and arrival_at and time.time() >= arrival_at:
                print(f'[EXT] Marking ARRIVED for {active}')
                post_mode(base_url, 'ARRIVED')
                mode = 'ARRIVED_WAIT_CONFIRM'
                arrival_at = None

            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print('[EXT] exiting')


if __name__ == '__main__':
    main()
