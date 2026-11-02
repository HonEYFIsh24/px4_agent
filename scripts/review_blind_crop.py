#!/usr/bin/env python3
"""Offline blinded ROI description on an archived real image; never flies.

The model receives only pixels strictly inside the original box and a generic
category vocabulary. It receives no query, label, box coordinates, full
image or previous response. This is an experiment, not a confirmation gate.
"""
import argparse
import base64
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from target_confirmation import review_crop

PROMPT = '''只描述提供的这张局部图中直接可见的像素；不要补造裁切之外的内容。
请从 vehicle、barrel、building_or_wall、ground、container、other、uncertain 中选择最能概括
可见主体的一个类别。目标不完整或不确定时选 uncertain；禁止根据局部灰板或反光标识猜测车辆。
报告实际可见的形状和边缘，以及明确可见的支持特征；看不到轮胎、车窗、油桶圆柱等特征时不要声称存在。
只输出 JSON：{"category":"上述一个类别","description":"实际局部像素","visible_features":["直接看到的特征"],"reason":"理由及不确定性"}。'''
ALLOWED = {'vehicle', 'barrel', 'building_or_wall', 'ground', 'container', 'other', 'uncertain'}
PROMPT_V2 = PROMPT.replace('vehicle、barrel、building_or_wall、ground、container、other、uncertain',
                           'vehicle、barrel、traffic_cone、building_or_wall、ground、container、other、uncertain')
ALLOWED_V2 = ALLOWED | {'traffic_cone'}
PROMPT_V3 = '''只描述提供的局部图中直接可见的像素，不补造裁切之外的内容。
分别检查 vehicle、barrel、traffic_cone、building_or_wall、ground、container、other，
可同时选择多个不同类别；只把图内可见结构足以辨认的类别加入 categories，不能根据上下文或用户目标猜测。
轮子及车身等证据可支持 vehicle；锥尖和橙白环带可支持 traffic_cone，锥不能当作油桶；
若没有任何可辨认物体则 categories 为空。只输出 JSON：
{"categories":["可见类别"],"description":"实际局部像素","visible_features":["直接看到的特征"],"reason":"理由及不确定性"}。'''


def prompt_for(variant):
    if variant == 'v1': return PROMPT, ALLOWED
    if variant == 'v2': return PROMPT_V2, ALLOWED_V2
    if variant == 'v3': return PROMPT_V3, ALLOWED_V2-{'uncertain'}
    raise ValueError('Unknown blinded prompt variant')


def validate_review(result, variant):
    _, allowed = prompt_for(variant)
    if not isinstance(result, dict): raise ValueError('Invalid blinded review schema')
    categories = result.get('categories') if variant == 'v3' else [result.get('category')]
    if (not isinstance(categories, list) or len(categories) > len(allowed)
            or any(not isinstance(x, str) or x not in allowed for x in categories) or len(set(categories)) != len(categories)
            or not all(isinstance(result.get(k), str) and 0 < len(result[k]) <= 1500 for k in ('description', 'reason'))
            or not isinstance(result.get('visible_features'), list) or len(result['visible_features']) > 12
            or any(not isinstance(x, str) or len(x) > 200 for x in result['visible_features'])):
        raise ValueError('Invalid blinded review schema')
    return result


def run(result_file, capture_id, target_index, output, variant='v1', manual_bbox=None):
    from agent_config import runtime_configuration
    from model_network import configure_agent_network
    from langchain_core.messages import HumanMessage, SystemMessage
    from langchain_openai import ChatOpenAI
    from vision import safe_error
    if Path(capture_id).name != capture_id or capture_id in ('.', '..'):
        raise ValueError('Invalid capture ID')
    record = json.loads(result_file.read_text()); command = record['command']
    report = next(p for p in command['perceptions'] if p.get('capture_id') == capture_id and p.get('status') == 'ok')
    if manual_bbox is None:
        if type(target_index) is not int or target_index < 0:
            raise ValueError('Target index must be a nonnegative integer')
        target = report['targets'][target_index]
        bbox_source = 'archived_report'
    else:
        from semantic_map import bbox
        target = {'label': None, 'bbox': bbox(manual_bbox)}
        bbox_source = 'manual_offline_evaluation_only'
    directory = Path(record.get('capture_directory') or result_file.parent/'captures')
    raw = (directory/(capture_id+'.jpg')).read_bytes()
    archive = json.loads((directory/(capture_id+'.json')).read_text())
    digest = hashlib.sha256(raw).hexdigest()
    if digest != archive['snapshot']['stereo_image_sha256']['left']:
        raise ValueError('Actual acquisition JPEG hash mismatch')
    if not any(c.get('status') == 'ok' and capture_id in c.get('capture_ids', []) and 'image' in c.get('input_modalities', [])
               for c in command['model_calls']):
        raise ValueError('Archive lacks actual model image-delivery evidence')
    frame = {'capture_id': capture_id, 'camera': {'image_sha256': digest},
             'image_base64': base64.b64encode(raw).decode()}
    crop = review_crop(frame, target['bbox'], padding=0.)
    output.mkdir(parents=True, exist_ok=False)
    (output/'source.jpg').write_bytes(raw)
    (output/'roi.jpg').write_bytes(base64.b64decode(crop['image_base64']))
    metadata = {k: v for k, v in crop.items() if k != 'image_base64'}
    metadata.update(source_result=str(result_file.resolve()), original_label=target['label'], original_bbox=target['bbox'],
                    input_scope='box pixels only; no target, coordinates, full frame or previous output',
                    actual_flight=False, production_gate=False, prompt_variant=variant, bbox_source=bbox_source)
    prompt, _ = prompt_for(variant)
    metadata['prompt_sha256'] = hashlib.sha256(prompt.encode()).hexdigest()
    configure_agent_network(ROOT/'.env')
    architecture, config, _ = runtime_configuration(ROOT/'.env')
    if architecture != 'single_vlm': raise ValueError('Requires existing single VLM configuration')
    started = time.monotonic()
    try:
        model = ChatOpenAI(**config, temperature=0, timeout=45, max_retries=0, max_tokens=700,
                           extra_body={'enable_thinking': False})
        response = model.invoke([SystemMessage(content=prompt), HumanMessage(content=[
            {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,'+crop['image_base64']}}])])
        text = response.content.strip()
        if text.startswith('```'): text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
        result = validate_review(json.loads(text), variant)
        metadata.update(status='ok', result=result, model=config['model'], token_usage=getattr(response, 'usage_metadata', None))
    except Exception as exc:
        metadata.update(status='error', **safe_error(exc, config))
    metadata['duration_seconds'] = round(time.monotonic()-started, 3)
    (output/'blind-review.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    return metadata


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true'); p.add_argument('--result', type=Path, required=True)
    p.add_argument('--capture-id', required=True); p.add_argument('--target-index', type=int, default=0)
    p.add_argument('--bbox', type=float, nargs=4, help='Offline evaluation ROI: normalized x_min y_min x_max y_max')
    p.add_argument('--variant', choices=('v1', 'v2', 'v3'), default='v1')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not args.execute:
        print('Dry run; no model request. --execute sends an archived simulator crop only.')
        sys.exit(0)
    result = run(args.result, args.capture_id, args.target_index, args.output, args.variant, args.bbox)
    print(json.dumps(result, ensure_ascii=False, indent=2)); sys.exit(0 if result['status'] == 'ok' else 1)
