"""Opt-in local NBV evidence. No flight actions, credentials, or reusable tickets."""
import json
import os
import threading
import time
import traceback
import uuid
from contextlib import contextmanager
from pathlib import Path

_lock = threading.Lock()


def sanitized(value):
    if isinstance(value, dict):
        return {k: ('[redacted-ticket]' if k in ('ticket', 'execution_ticket') else sanitized(v))
                for k, v in value.items() if k not in ('api_key', 'image_base64', 'authorization', 'cookie')}
    if isinstance(value, (list, tuple)):
        return [sanitized(v) for v in value]
    return value


def exception_detail(exc):
    # No locals or source text: those may contain private provider configuration.
    return {'exception_type': type(exc).__name__, 'message': str(exc)[:1000],
            'frames': [{'module': Path(f.filename).name, 'function': f.name, 'line': f.lineno}
                       for f in traceback.extract_tb(exc.__traceback__)]}


def emit(event):
    directory = os.getenv('UAV_CAPTURE_EVIDENCE_DIR')
    if not directory:
        return
    try:
        value = {'wall_s': time.time(), 'pid': os.getpid(), **sanitized(event)}
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n'
        with _lock:
            with (Path(directory)/('nbv-diagnostics-'+str(os.getpid())+'.jsonl')).open('a') as stream:
                stream.write(encoded)
    except (OSError, TypeError, ValueError):
        # A diagnostic failure must never replace the original flight exception.
        pass


@contextmanager
def stage(name, inputs):
    identifier = uuid.uuid4().hex
    emit({'event': 'begin', 'stage': name, 'id': identifier, 'inputs': inputs})
    output = {}
    try:
        yield output
    except Exception as exc:
        emit({'event': 'error', 'stage': name, 'id': identifier, **exception_detail(exc)})
        raise
    else:
        emit({'event': 'end', 'stage': name, 'id': identifier, **output})
