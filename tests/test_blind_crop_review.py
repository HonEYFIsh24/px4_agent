import unittest

from review_blind_crop import prompt_for, validate_review


class BlindReviewTests(unittest.TestCase):
    def test_multilabel_and_empty_result_are_distinct_valid_observations(self):
        base = {'description': 'pixels', 'reason': 'limited visibility', 'visible_features': []}
        for categories in ([], ['vehicle'], ['vehicle', 'traffic_cone']):
            self.assertEqual(validate_review({**base, 'categories': categories}, 'v3')['categories'], categories)
        for bad in (['vehicle', 'vehicle'], ['unknown'], ['uncertain'], [None], 'vehicle'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_review({**base, 'categories': bad}, 'v3')

    def test_legacy_schema_and_invalid_response_are_rejected(self):
        base = {'description': 'pixels', 'reason': 'limited visibility', 'visible_features': []}
        self.assertEqual(validate_review({**base, 'category': 'uncertain'}, 'v1')['category'], 'uncertain')
        for result in (None, [], {**base, 'category': 'traffic_cone'},
                       {**base, 'category': 'vehicle', 'description': ''},
                       {**base, 'category': 'vehicle', 'visible_features': [{}]}):
            with self.subTest(result=result), self.assertRaises(ValueError):
                validate_review(result, 'v1')
        with self.assertRaises(ValueError): prompt_for('unapproved')


if __name__ == '__main__': unittest.main()
