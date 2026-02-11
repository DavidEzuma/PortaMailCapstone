from datetime import datetime
from interface.contract import make_event

EVENT_LIMIT = 50
_events = []


def add_event(name, payload=None):
    evt = make_event(datetime.now().astimezone().isoformat(), name, payload)
    _events.append(evt)
    if len(_events) > EVENT_LIMIT:
        del _events[0 : len(_events) - EVENT_LIMIT]
    return evt


def read_events():
    return list(_events)
