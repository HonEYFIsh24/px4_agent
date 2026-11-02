#!/usr/bin/env python3
"""Replay an actual archived pair with the agent's exact source/current ROI inputs; never flies."""
import argparse
import base64
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from target_confirmation import observation_for, observations_for, prepare_review_images, PAIRED_REVIEW_PROMPT, pair_geometry


def run(result_file, event_index, output):
    from agent_config import runtime_configuration
    from model_network import configure_agent_network
    from langchain_openai import ChatOpenAI
    from langchain_core.messages import HumanMessage, SystemMessage
    from vision import safe_error
    record = json.loads(result_file.read_text())
    command = record['command']
    events = [e for e in command['target_confirmation_events'] if e['outcome'] in ('confirmed', 'unconfirmed', 'rejected')]
    event = events[event_index]; active = event['candidate']
    directory = Path(record.get('capture_directory') or result_file.parent/'captures')
    frames = []
    for identifier in (active['source_capture_id'], active['current_capture_id']):
        if Path(identifier).name != identifier or identifier in ('.', '..'):
            raise ValueError('Invalid replay capture basename')
        archive = json.loads((directory/(identifier+'.json')).read_text())
        raw = (directory/(identifier+'.jpg')).read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest != archive['snapshot']['stereo_image_sha256']['left']:
            raise ValueError('Actual pair JPEG hash differs from acquisition metadata')
        if not any(c.get('status') == 'ok' and identifier in c.get('capture_ids', [])
                   and 'image' in c.get('input_modalities', []) for c in command['model_calls']):
            raise ValueError('Archived image lacks model-delivery evidence')
        frames.append({'capture_id': identifier, 'camera': {'image_sha256': digest},
                       'image_base64': base64.b64encode(raw).decode()})
    snapshot = json.loads((directory/(active['current_capture_id']+'.semantic-map.json')).read_text())
    if event['owner'] != snapshot['owner'] or event['owner'][1] != command['id']:
        raise ValueError('Archived pair ownership differs from task/map')
    semantic_map = SimpleNamespace(landmarks={l['id']: SimpleNamespace(**l) for l in snapshot['landmarks']})
    source_landmark, original = observation_for(semantic_map, active['target_id'], active['source_capture_id'])
    candidates = observations_for(semantic_map, event['query'], active['current_capture_id'])
    output.mkdir(parents=True, exist_ok=False)
    content, metadata = prepare_review_images(*frames, original['bbox'], [(l.id, o['bbox']) for l, o in candidates], output)
    for name, frame in zip(('source.jpg', 'current.jpg'), frames):
        (output/name).write_bytes(base64.b64decode(frame['image_base64']))
    metadata.update(query=event['query'], source_capture_id=frames[0]['capture_id'], current_capture_id=frames[1]['capture_id'],
        current_target_ids=[l.id for l, _ in candidates], source_result=str(result_file.resolve()), actual_flight=False,
        geometry_hypotheses=[{'target_id': l.id, **pair_geometry(source_landmark, original, l, o)} for l, o in candidates])
    configure_agent_network(ROOT/'.env')
    architecture, config, _ = runtime_configuration(ROOT/'.env')
    if architecture != 'single_vlm':
        raise ValueError('Replay requires configured single VLM')
    started = time.monotonic()
    try:
        model = ChatOpenAI(**config, temperature=0, timeout=45, max_retries=0, max_tokens=1200,
                           extra_body={'enable_thinking': False})
        # Do not show geometry hypotheses or prior review conclusions to the VLM.
        visual_metadata = {k: metadata[k] for k in ('query', 'source_capture_id', 'current_capture_id', 'current_target_ids')}
        response = model.invoke([SystemMessage(content=PAIRED_REVIEW_PROMPT+
            '只输出JSON，字段source_supported/current_supported/same_object为布尔值，current_target_id为空或提供的ID，'
            'source_description/current_description/reason为字符串。'), HumanMessage(content=[
                {'type': 'text', 'text': json.dumps(visual_metadata, ensure_ascii=False)}, *content])])
        text = response.content.strip()
        if text.startswith('```'):
            text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
        result = json.loads(text)
        if any(type(result.get(k)) is not bool for k in ('source_supported', 'current_supported', 'same_object')):
            raise ValueError('Review flags are not booleans')
        if result.get('current_target_id') not in ['', *metadata['current_target_ids']]:
            raise ValueError('Review selected an unknown candidate')
        if not all(isinstance(result.get(k), str) and 0 < len(result[k]) <= 1500
                   for k in ('source_description', 'current_description', 'reason')):
            raise ValueError('Review lacks actual pixel descriptions')
        metadata.update(status='ok', result=result, model=config['model'], token_usage=getattr(response, 'usage_metadata', None))
    except Exception as exc:
        metadata.update(status='error', **safe_error(exc, config))
    metadata['duration_seconds'] = round(time.monotonic()-started, 3)
    (output/'review.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--event-index', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('Dry run; add --execute for an archived paired-image model request. No flight.')
        sys.exit(0)
    result = run(args.result, args.event_index, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result['status'] == 'ok' else 1)
