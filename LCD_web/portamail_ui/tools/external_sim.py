import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

BASE_URL = 'http://127.0.0.1:5050'
POLL_SEC = 1.0

_seen = []
_seen_limit = 200
_stop = False
_last_ts = None


def _event_key(evt):
    if not isinstance(evt, dict):
        return None
    ts = evt.get('ts')
    name = evt.get('name') or evt.get('type')
    if ts and name:
        return f'{ts}:{name}'
    return None


def _http_get(path):
    url = f'{BASE_URL}{path}'
    with urllib.request.urlopen(url, timeout=5) as resp:
        if resp.status != 200:
            raise RuntimeError(f'GET {url} failed: {resp.status}')
        return resp.read().decode('utf-8')


def _http_post(path, body):
    url = f'{BASE_URL}{path}'
    data = json.dumps(body).encode('utf-8')
    req = urllib.request.Request(url, data=data, method='POST')
    req.add_header('Content-Type', 'application/json')
    with urllib.request.urlopen(req, timeout=5) as resp:
        if resp.status != 200:
            raise RuntimeError(f'POST {url} failed: {resp.status}')
        return resp.read().decode('utf-8')


def _poll_events_loop():
    global _stop, _last_ts
    while not _stop:
        try:
            path = '/api/events'
            if _last_ts:
                query = urllib.parse.urlencode({'since_ts': _last_ts})
                path = f'{path}?{query}'
            raw = _http_get(path)
            events = json.loads(raw)
            if isinstance(events, list):
                for evt in events:
                    key = _event_key(evt)
                    if key is None:
                        continue
                    if key in _seen:
                        continue
                    _seen.append(key)
                    if len(_seen) > _seen_limit:
                        _seen[:] = _seen[-_seen_limit:]
                    ts = evt.get('ts')
                    if ts:
                        _last_ts = ts
                    print(json.dumps(evt))
        except Exception as exc:
            print(f'[sim] events poll error: {exc}')
        time.sleep(POLL_SEC)


def _cmd_arrived():
    try:
        _http_post('/api/mode', {'mode': 'ARRIVED'})
        print('[sim] mode set: ARRIVED')
    except Exception as exc:
        print(f'[sim] failed to set ARRIVED: {exc}')


def _cmd_dock():
    try:
        _http_post('/api/mode', {'mode': 'DOCK_IDLE'})
        print('[sim] mode set: DOCK_IDLE')
    except Exception as exc:
        print(f'[sim] failed to set DOCK_IDLE: {exc}')


def _cmd_state():
    try:
        raw = _http_get('/api/state')
        st = json.loads(raw)
        screen = st.get('screen')
        mode = st.get('mode')
        active = st.get('active_room')
        pending = st.get('pending_rooms')
        print(f'[sim] mode={mode} screen={screen} active_room={active} pending_rooms={pending}')
    except Exception as exc:
        print(f'[sim] state fetch error: {exc}')


def main():
    thread = threading.Thread(target=_poll_events_loop, daemon=True)
    thread.start()

    print('external_sim ready: commands = arrived | dock | state | quit')
    while True:
        try:
            cmd = input('> ').strip().lower()
        except EOFError:
            cmd = 'quit'
        if cmd == 'arrived':
            _cmd_arrived()
        elif cmd == 'dock':
            _cmd_dock()
        elif cmd == 'state':
            _cmd_state()
        elif cmd == 'quit':
            break
        elif cmd == '':
            continue
        else:
            print('[sim] unknown command')

    global _stop
    _stop = True
    thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
