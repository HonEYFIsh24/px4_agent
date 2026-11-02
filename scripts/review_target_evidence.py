#!/usr/bin/env python3
"""Single-image ROI review experiment on an archived report; never flies.

This tests a visual-review prompt, not multiview confirmation or class accuracy.
Original reports and semantic-map measurements remain unchanged.
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

REVIEW_PROMPT = '''你是视觉证据复核器。只依据本次提供的完整图像和由检测框裁出的像素判断，
不能把原报告、对象名称、置信度或双目成功当作对象真实存在的证明。
先描述框内实际可见内容，再判断这些像素是否足以支持给定对象名称及属性。
缺乏可见证据时 source_supported=false，不用常识补造隐藏物体。
裁图只是同一张照片的放大，不是另一视角。只输出 JSON：
{"source_supported":布尔值,"object_description":"实际像素内容","reason":"依据及局限"}。'''


def run(result_path, capture_id, target_index, output, env_file):
    from langchain_core.messages import SystemMessage, HumanMessage
    from langchain_openai import ChatOpenAI
    from agent_config import runtime_configuration
    from model_network import configure_agent_network
    from vision import safe_error
    record = json.loads(result_path.read_text())
    report = next(r for r in record['command']['perceptions'] if r['capture_id'] == capture_id)
    target = report['targets'][target_index]
    directory = Path(record.get('capture_directory') or result_path.parent/'captures')
    if Path(capture_id).name != capture_id or capture_id in ('.', '..'):
        raise ValueError('Invalid capture basename')
    archive = json.loads((directory/(capture_id+'.json')).read_text())
    raw = (directory/(capture_id+'.jpg')).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != archive['snapshot']['stereo_image_sha256']['left']:
        raise ValueError('Archived source image hash mismatch')
    if not any(m.get('status') == 'ok' and capture_id in m.get('capture_ids', [])
               and 'image' in m.get('input_modalities', []) for m in record['command']['model_calls']):
        raise ValueError('Source report lacks actual model image delivery evidence')
    frame = {'capture_id': capture_id, 'camera': {'image_sha256': digest},
             'image_base64': base64.b64encode(raw).decode()}
    crop = review_crop(frame, target['bbox'])
    output.mkdir(parents=True, exist_ok=False)
    (output/'source.jpg').write_bytes(raw)
    (output/'roi.jpg').write_bytes(base64.b64decode(crop['image_base64']))
    metadata = {k: v for k, v in crop.items() if k != 'image_base64'}
    metadata.update(label_hypothesis=target['label'], normalized_bbox=target['bbox'],
                    source_result=str(result_path.resolve()), actual_flight=False,
                    evidence_scope='single archived image with derived ROI; not multiview evidence')
    configure_agent_network(env_file)
    architecture, config, _ = runtime_configuration(env_file)
    if architecture != 'single_vlm':
        raise ValueError('Review experiment requires the existing single VLM configuration')
    options = {'temperature': 0, 'timeout': 45, 'max_retries': 0, 'max_tokens': 900}
    if 'aliyuncs.com' in (config.get('base_url') or ''):
        options['extra_body'] = {'enable_thinking': False}
    started = time.monotonic()
    try:
        model = ChatOpenAI(**config, **options)
        response = model.invoke([SystemMessage(REVIEW_PROMPT), HumanMessage(content=[
            {'type': 'text', 'text': json.dumps({'label_hypothesis': target['label'],
                'source_bbox': target['bbox'], 'images': ['完整实际照片', '同图检测框附近的裁图']}, ensure_ascii=False)},
            {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,'+frame['image_base64']}},
            {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,'+crop['image_base64']}},
        ])])
        text = response.content.strip()
        if text.startswith('```'):
            text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
        result = json.loads(text)
        if type(result.get('source_supported')) is not bool or not all(
                isinstance(result.get(k), str) and 0 < len(result[k]) <= 1500 for k in ('object_description', 'reason')):
            raise ValueError('Invalid review result schema')
        metadata.update(status='ok', result=result, model=config['model'],
                        token_usage=getattr(response, 'usage_metadata', None))
    except Exception as exc:
        metadata.update(status='error', **safe_error(exc, config))
    metadata['duration_seconds'] = round(time.monotonic()-started, 3)
    (output/'review.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False))
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--capture-id', required=True)
    parser.add_argument('--target-index', type=int, default=0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--env-file', type=Path, default=ROOT/'.env')
    args = parser.parse_args()
    if not args.execute:
        print('Dry run; no model request, capture or flight. Add --execute for archived image review.')
        raise SystemExit(0)
    result = run(args.result, args.capture_id, args.target_index, args.output, args.env_file)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['status'] == 'ok' else 1)
