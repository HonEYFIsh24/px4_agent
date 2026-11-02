#!/usr/bin/env python3
"""Experiment: ask configured VLM for visible polygon inside an archived target box."""
import argparse
import base64
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def run(result_file, capture_id, target_index, output):
    from agent_config import runtime_configuration
    from model_network import configure_agent_network
    from langchain_core.messages import HumanMessage, SystemMessage
    from langchain_openai import ChatOpenAI
    from vision import safe_error
    record = json.loads(result_file.read_text()); command = record['command']
    if Path(capture_id).name != capture_id or capture_id in ('.', '..'):
        raise ValueError('Invalid capture ID')
    report = next(p for p in command['perceptions'] if p.get('capture_id') == capture_id and p.get('status') == 'ok')
    target = report['targets'][target_index]; box = target['bbox']
    directory = Path(record.get('capture_directory') or result_file.parent/'captures')
    raw = (directory/(capture_id+'.jpg')).read_bytes()
    archive = json.loads((directory/(capture_id+'.json')).read_text())
    digest = hashlib.sha256(raw).hexdigest()
    if digest != archive['snapshot']['stereo_image_sha256']['left']:
        raise ValueError('Image hash mismatch')
    if not any(c.get('status') == 'ok' and capture_id in c.get('capture_ids', []) and 'image' in c.get('input_modalities', [])
               for c in command['model_calls']):
        raise ValueError('No actual model delivery for this capture')
    output.mkdir(parents=True, exist_ok=False)
    (output/'source.jpg').write_bytes(raw)
    configure_agent_network(ROOT/'.env')
    architecture, config, _ = runtime_configuration(ROOT/'.env')
    if architecture != 'single_vlm':
        raise ValueError('Requires configured single VLM')
    result = {'source_result': str(result_file.resolve()), 'capture_id': capture_id, 'image_sha256': digest,
              'target_label_hypothesis': target['label'], 'original_bbox': box,
              'actual_flight': False, 'meaning': 'uncalibrated visual hypothesis; not a ground-truth segmentation'}
    started = time.monotonic()
    try:
        model = ChatOpenAI(**config, temperature=0, timeout=45, max_retries=0, max_tokens=1200,
                           extra_body={'enable_thinking': False})
        instruction = ('只看实际图片，检查给定框内是否有目标可辨认的可见表面。不要推测被遮挡部分，'
            '不要把地面、墙、交通锥或相邻物体纳入目标。框外目标不可填入框内。'
            '如果框内不能辨认目标，visible=false且polygon=[]。如果可辨认，仅围绕框内目标的可见部分，'
            '给3到24个有序顶点，坐标归一化到整张图0到1，顶点必须在原框内。'
            '只输出JSON：{"visible":布尔值,"polygon":[[x,y],...],"description":"框内像素","reason":"理由"}。')
        response = model.invoke([SystemMessage(content=instruction), HumanMessage(content=[
            {'type': 'text', 'text': json.dumps({'target': target['label'], 'bbox': box}, ensure_ascii=False)},
            {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,'+base64.b64encode(raw).decode()}}])])
        text = response.content.strip()
        if text.startswith('```'):
            text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
        value = json.loads(text); points = value.get('polygon')
        if type(value.get('visible')) is not bool or not isinstance(points, list):
            raise ValueError('Invalid polygon schema')
        if value['visible'] and not 3 <= len(points) <= 24 or not value['visible'] and points:
            raise ValueError('Polygon does not match visibility')
        if any(not isinstance(p, list) or len(p) != 2 or any(type(v) not in (int, float) for v in p)
               or not box[0] <= p[0] <= box[2] or not box[1] <= p[1] <= box[3] for p in points):
            raise ValueError('Polygon vertices must be inside the original box')
        if not all(isinstance(value.get(k), str) and 0 < len(value[k]) <= 1500 for k in ('description', 'reason')):
            raise ValueError('Missing visual description')
        result.update(status='ok', result=value, model=config['model'], token_usage=getattr(response, 'usage_metadata', None))
    except Exception as exc:
        result.update(status='error', **safe_error(exc, config))
    result['duration_seconds'] = round(time.monotonic()-started, 3)
    (output/'region-review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--capture-id', required=True)
    parser.add_argument('--target-index', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('Dry run. Add --execute to send an archived simulator image to the configured model; no flight.')
        sys.exit(0)
    result = run(args.result, args.capture_id, args.target_index, args.output)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    sys.exit(0 if result['status'] == 'ok' else 1)
