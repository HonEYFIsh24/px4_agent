import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from evaluate_blind_crop_holdout import summarize, validate_manifest
from review_blind_crop import prompt_for


class HoldoutSummaryTests(unittest.TestCase):
    def setUp(self):
        self.manifest = {'prompt_variant': 'v3', 'prompt_sha256': 'prompt',
                         'selection_limitations': 'fixture only', 'annotation_source': 'controlled'}
        self.cases = [dict(id='a', capture_id='a', source_image_sha256='a', target_category='vehicle', visually_present=True),
                      dict(id='b', capture_id='b', source_image_sha256='b', target_category='barrel', visually_present=True),
                      dict(id='c', capture_id='c', source_image_sha256='c', target_category='barrel', visually_present=False)]
        def review(identifier, labels):
            return {'status': 'ok', 'source_capture_id': identifier, 'source_image_sha256': identifier,
                    'prompt_variant': 'v3', 'prompt_sha256': 'prompt', 'duration_seconds': 2.,
                    'result': {'categories': labels, 'description': 'pixels', 'reason': 'uncertain', 'visible_features': []}}
        self.reviews = {'a': review('a', ['vehicle']), 'b': review('b', []), 'c': review('c', ['traffic_cone'])}

    def test_empty_and_other_class_are_not_conflated_with_confirmed_absence(self):
        result = summarize(self.cases, self.reviews, self.manifest)
        self.assertEqual(result['counts']['positive_supported'], 1)
        self.assertEqual(result['counts']['positive_not_supported'], 1)
        self.assertEqual(result['counts']['negative_falsely_supported'], 0)
        self.assertEqual(result['counts']['empty_outputs'], 1)
        self.assertIn('not an explicit absence', result['semantics'])

    def test_changed_provenance_or_failed_response_cannot_be_counted(self):
        for field, value in (('source_image_sha256', 'wrong'), ('prompt_sha256', 'wrong'), ('status', 'error')):
            with self.subTest(field=field):
                reviews = copy.deepcopy(self.reviews); reviews['a'][field] = value
                with self.assertRaises(ValueError): summarize(self.cases, reviews, self.manifest)

    def test_manifest_rejects_duplicate_development_image_and_changed_prompt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); captures = root/'captures'; captures.mkdir()
            (captures/'image.jpg').write_bytes(b'controlled test bytes')
            (root/'result.json').write_text(json.dumps({'capture_directory': str(captures)}))
            digest = hashlib.sha256(b'controlled test bytes').hexdigest()
            prompt = hashlib.sha256(prompt_for('v3')[0].encode()).hexdigest()
            case = {'id': 'case', 'capture_id': 'image', 'source_result': 'result.json',
                    'source_image_sha256': digest, 'target_category': 'vehicle',
                    'visually_present': True, 'target_index': 0}
            manifest = {'prompt_variant': 'v3', 'prompt_sha256': prompt,
                        'excluded_development_image_sha256': [], 'cases': [case]}
            with mock.patch('evaluate_blind_crop_holdout.ROOT', root):
                self.assertEqual(len(validate_manifest(manifest)), 1)
                for changed in ('excluded', 'duplicate', 'prompt', 'capture'):
                    item = copy.deepcopy(manifest)
                    if changed == 'excluded': item['excluded_development_image_sha256'].append(digest)
                    if changed == 'duplicate': item['cases'].append({**case, 'id': 'another'})
                    if changed == 'prompt': item['prompt_sha256'] = 'invalid'
                    if changed == 'capture': item['cases'][0]['capture_id'] = '../image'
                    with self.subTest(changed=changed), self.assertRaises(ValueError):
                        validate_manifest(item)


if __name__ == '__main__': unittest.main()
