"""Synthetic records exercise evaluation integrity; they are not Gazebo evidence."""
import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from visual_eval_dataset import validate, score


class VisualEvalTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        (self.root/'world').write_bytes(b'synthetic-world'); world=hashlib.sha256(b'synthetic-world').hexdigest()
        seed={'seed':17,'source':'actual_gzserver_proc_cmdline'}
        images=[]
        for i,state in enumerate(('identifiable','absent')):
            raw=('synthetic-image-'+str(i)).encode();digest=hashlib.sha256(raw).hexdigest();capture='capture'+str(i)
            (self.root/(capture+'.jpg')).write_bytes(raw)
            geometry={'image_sha256':{'left':digest},'world_sha256':world,'body_pose_enu':{'position_m':[i,0,3]},'camera_origin_enu_m':[i,0,3], 'camera_intrinsics':{'fx':320},'stamp_sim_s':i+1,'sortie_id':'flight'}
            (self.root/(capture+'.json')).write_text(json.dumps(geometry))
            label={'state':state,'category':'barrel','observation_region':[0,0,1,1],
                'target_bbox':[.2,.2,.4,.4] if i==0 else None,'instance_id':'obj' if i==0 else None,
                'occlusion':'partial' if i==0 else 'none','distance_band':'unknown','annotator':'alice','reviewer':'bob',
                'annotation_source':'human_visual_review','reviewer_kind':'human','review_status':'approved'}
            images.append({'id':capture,'capture_id':capture,'image_path':capture+'.jpg','geometry_path':capture+'.json',
                'world_path':'world','source_result':'result.json','image_sha256':digest,'world_sha256':world,
                'scene_id':'scene','layout_group':'layout','split':'holdout','run':'run', 'seed_evidence':seed,
                'body_pose_enu':geometry['body_pose_enu'],'camera_origin_enu_m':geometry['camera_origin_enu_m'],
                'camera_intrinsics':geometry['camera_intrinsics'],'stamp_sim_s':i+1,'sortie_id':'flight','label':label})
        (self.root/'result.json').write_text(json.dumps({'run':'run','gazebo_seed_evidence':seed,'command':{'perceptions':[{'capture_id':x['id'],'status':'ok'} for x in images]}}))
        self.data={'schema_version':1,'images':images,'pairs':[],'tasks':[]}

    def test_provenance_and_review_are_required(self):
        self.assertEqual(validate(self.data,self.root,False)['images'],2)
        bad=copy.deepcopy(self.data);bad['images'][0]['label']['reviewer']='alice'
        with self.assertRaisesRegex(ValueError,'independent'):validate(bad,self.root,False)
        bad=copy.deepcopy(self.data);bad['images'][0]['seed_evidence']['seed']=99
        with self.assertRaisesRegex(ValueError,'evidence mismatch'):validate(bad,self.root,False)
        bad=copy.deepcopy(self.data);bad['images'][0]['camera_origin_enu_m']=[99,0,0]
        with self.assertRaisesRegex(ValueError,'camera origin'):validate(bad,self.root,False)
        (self.root/'capture0.jpg').write_bytes(b'tampered')
        with self.assertRaisesRegex(ValueError,'hash mismatch'):validate(self.data,self.root,False)

    def test_split_leakage_by_world_is_rejected_even_with_renamed_scene(self):
        bad=copy.deepcopy(self.data);bad['images'][1].update(scene_id='alias',layout_group='different-name',split='development')
        with self.assertRaisesRegex(ValueError,'leaks'):validate(bad,self.root,False)

    def test_insufficient_is_not_absent_or_false_negative(self):
        self.data['images'][0]['label'].update(state='insufficient_evidence',target_bbox=None,instance_id=None)
        validate(self.data,self.root,False)
        pred={'images':[{'id':'capture0','answer':'uncertain'},{'id':'capture1','answer':'present'}], 'pairs':[], 'tasks':[]}
        report=score(self.data,pred)['holdout']
        self.assertEqual(report['identifiable_false_negative_explicit']['denominator'],0)
        self.assertEqual(report['single_frame_false_positive']['rate'],1.)
        self.assertEqual(report['single_frame_undecided']['rate'],.5)

    def test_no_full_claim_from_small_dataset_or_missing_predictions(self):
        with self.assertRaisesRegex(ValueError,'80-120'): validate(self.data,self.root)
        with self.assertRaisesRegex(ValueError,'do not match'):score(self.data,{'images':[],'pairs':[],'tasks':[]})

    def test_pairs_require_independent_identity_and_consistent_instances(self):
        label=self.data['images'][1]['label'];label.update(state='identifiable',target_bbox=[.2,.2,.4,.4],instance_id='other')
        pair={'id':'p','image_a':'capture0','image_b':'capture1','identity':'different','annotator':'alice','reviewer':'bob','annotation_source':'human_visual_review','reviewer_kind':'human','review_status':'approved'}
        self.data['pairs']=[pair];validate(self.data,self.root,False)
        pair['identity']='same'
        with self.assertRaisesRegex(ValueError,'contradicts'):validate(self.data,self.root,False)

    def test_task_metric_excludes_unresolved_confirmations_from_adjudicated_denominator(self):
        base={'split':'holdout','image_ids':['capture0'],'annotator':'alice','reviewer':'bob','annotation_source':'human_visual_review','reviewer_kind':'human','review_status':'approved'}
        for task in ('one','two'):
            (self.root/(task+'.json')).write_text(json.dumps({'command':{'id':task,'status':'succeeded',
                'perceptions':[{'capture_id':'capture0','status':'ok'}],'search_outcome':{'target_confirmed':True}}}))
        self.data['tasks']=[dict(base,id='t1',task_id='one',source_result='one.json',actual_outcome='confirmed',identity_label='incorrect'),
                            dict(base,id='t2',task_id='two',source_result='two.json',actual_outcome='confirmed',identity_label='insufficient_evidence')]
        validate(self.data,self.root,False)
        predictions={'images':[{'id':'capture0','answer':'absent'},{'id':'capture1','answer':'absent'}], 'pairs':[], 'tasks':[{'id':'t1','answer':'confirmed'},{'id':'t2','answer':'confirmed'}]}
        report=score(self.data,predictions)['holdout']
        self.assertEqual(report['identifiable_false_negative_explicit']['rate'],1.)
        self.assertEqual(report['task_incorrect_confirmation_among_adjudicated_confirmed'],{'numerator':1,'denominator':1,'rate':1.})
        self.assertEqual(report['task_confirmed_unadjudicated'],1)
        predictions['tasks'][0]['answer']='not_confirmed'
        with self.assertRaisesRegex(ValueError,'archived outcomes'):score(self.data,predictions)


if __name__=='__main__': unittest.main()
