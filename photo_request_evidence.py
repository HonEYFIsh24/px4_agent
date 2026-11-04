"""Opt-in local SDK request evidence; never performs model/flight calls itself."""
import base64
import hashlib
import json
import os
import re
import time
import uuid
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path

from langchain_openai import ChatOpenAI

_active = ContextVar('photo_request_evidence', default=None)
_ROOT = Path(__file__).resolve().parent


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)


def sanitized(value, secrets, images, changes):
    if isinstance(value, dict):
        result={}
        for key,item in value.items():
            if key.lower() in ('api_key','authorization','cookie','password','ticket','execution_ticket'):
                changes.append(key); result[key]='[redacted]'
            else: result[key]=sanitized(item,secrets,images,changes)
        return result
    if isinstance(value, (list,tuple)):
        return [sanitized(v,secrets,images,changes) for v in value]
    if isinstance(value, str):
        if value.startswith('data:image/jpeg;base64,'):
            raw = base64.b64decode(value.split(',',1)[1], validate=True)
            if len(raw)>3000000 or not raw.startswith(b'\xff\xd8') or not raw.endswith(b'\xff\xd9'):
                raise ValueError('invalid diagnostic JPEG')
            digest=hashlib.sha256(raw).hexdigest()
            images[digest]=dict(sha256=digest,mime_type='image/jpeg',bytes=len(raw))
            return 'local-image-sha256:'+digest
        if 'data:image' in value or 'base64,' in value:
            changes.append('embedded_image'); return '[redacted-embedded-image]'
        for secret in secrets:
            if secret and secret in value:
                value=value.replace(secret,'[redacted-secret]'); changes.append('known_secret')
        cleaned=re.sub(r'(?i)\bBearer\s+[^\s"\x27,;]+','Bearer [redacted]',value)
        cleaned=re.sub(r'(https?://)[^/\s@]+@',r'\1[redacted]@',cleaned)
        cleaned=re.sub(r'(?i)("(?:api_key|authorization|cookie|password|ticket|execution_ticket)"\s*:\s*)"(?:\\.|[^"\\])*"',
                       r'\1"[redacted]"',cleaned)
        if cleaned!=value: changes.append('credential_text')
        return cleaned
    if value is None or type(value) in (int,float,bool): return value
    raise TypeError('non-JSON diagnostic input')


def write_private(directory, name, value):
    raw=canonical(value).encode('utf-8')
    if len(raw)>4000000: raise ValueError('diagnostic record too large')
    directory.mkdir(parents=True,exist_ok=True)
    descriptor=os.open(str(directory/name),os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(descriptor,'wb') as stream: stream.write(raw)


@contextmanager
def request_evidence(enabled, owner, capture_id, model_config):
    """Scope only a photo planning invocation, including its returned model output.

    Any disk/serialization failure is diagnostic-only and cannot change flight
    behavior. HTTP headers, URL, proxy settings and provider credentials are never
    taken from the model. Image bytes remain in the existing capture evidence.
    """
    directory=os.getenv('UAV_PHOTO_REQUEST_EVIDENCE_DIR') if enabled else None
    state={'status':'disabled','requests':[]}
    if directory:
        try:
            path=Path(directory).resolve(); path.relative_to((_ROOT/'logs').resolve())
            state.update(status='enabled',directory=path,owner=list(owner),capture_id=capture_id,
                         secrets=[str(model_config.get('api_key') or '')])
        except (OSError,ValueError,TypeError): state['status']='unavailable'
    token=_active.set(state if state['status']=='enabled' else None)
    try:
        yield state
    finally:
        _active.reset(token)


def record_response(state, messages=None, failed=False):
    if state['status'] not in ('enabled','recorded'): return
    try:
        changes=[];images={}
        response=[{'type':m.type,'content':m.content,'tool_calls':getattr(m,'tool_calls',[])} for m in (messages or [])]
        value=sanitized(response,state['secrets'],images,changes)
        for request in state['requests']:
            write_private(state['directory'],request['id']+'.response.json',dict(
                request_id=request['id'],failed=failed,response=value,redactions=changes))
        state['status']='recorded' if state['requests'] else 'unavailable'
    except (OSError,ValueError,TypeError,AttributeError): state['status']='unavailable'


class EvidenceChatOpenAI(ChatOpenAI):
    """Preserve parent SDK parameters exactly; archive an image-referenced copy."""
    def _get_request_payload(self, input_, *, stop=None, **kwargs):
        payload=super()._get_request_payload(input_,stop=stop,**kwargs)
        state=_active.get()
        if state is not None:
            try:
                changes=[];images={}; identifier=uuid.uuid4().hex
                safe=sanitized(payload,state['secrets'],images,changes)
                value=dict(schema_version=1,id=identifier,wall_s=time.time(),owner=state['owner'],
                    capture_id=state['capture_id'],kind='langchain_openai_sdk_chat_parameters',
                    transport_headers_recorded=False,flight_execution_authority=False,
                    payload_sha256=hashlib.sha256(canonical(payload).encode('utf-8')).hexdigest(),
                    payload=safe,images=list(images.values()),redactions=changes,
                    exact_payload_reconstructable=not changes)
                write_private(state['directory'],identifier+'.request.json',value)
                state['requests'].append(dict(id=identifier,capture_id=state['capture_id'],
                                             exact_payload_reconstructable=not changes))
            except (OSError,ValueError,TypeError): state['status']='unavailable'
        return payload
