"""Stage-A real Agent graph with local model/executor fixtures; no flight IO."""
import copy
import json
import math
import os
import time
import unittest
from unittest import mock

from mission import MissionError
from task_runtime.contracts import TaskLimits
from task_runtime.planning import compile_plan, target_query, plan_constraints, plan_request_schema
from task_runtime.orchestrator import TaskOrchestrator
from search_limits import MAX_OBSERVATIONS
from viewpoint_geometry import registered_identity
from search_tools import coordinate_epoch
import test_nbv_search_integration as nbv_fixtures
from test_nbv_search_integration import ScopedFixtureBridge
from test_single_vlm import tool_call
from test_target_observation import measured_stereo
from coverage_search import project_ground_point
import test_viewpoint_authority as authority_fixtures


def proposal(close=False, relative=False):
    stages = [{'kind':'search','target_ref':'target'}]
    if close:
        stages += [{'kind':'close_photo','target_ref':'target'}]
    stages += [{'kind':'report','target_ref':'target'}]
    stages += ([{'kind':'relative_land','target_ref':'target','relation':'object_front'}] if relative else
               [{'kind':'return_home_land'}])
    return {'targets':[{'ref':'target','label':'蓝色油桶','attributes':{}}], 'stages':stages}


class RuntimeBridge(ScopedFixtureBridge):
    def request(self, path, payload=None, **kwargs):
        if path == '/navigation/stage-transition':
            if (self.telemetry.get('mission') or {}).get('status') == 'running':
                raise MissionError('running fixture flight')
            return self.authority.advance_stage(payload['owner'],payload['stage_id'],payload['revision'],
                registered_identity(self.nav,coordinate_epoch(self.telemetry)))
        return super().request(path,payload,**kwargs)


def current_report(messages, point=None, malformed=False):
    message = next(m for m in reversed(messages) if getattr(m,'name',None)=='observe_target_search_view')
    capture = json.loads(message.content)
    box = [.2,.3,.4,.6]
    if point is not None:
        # The script is a fixture for state transitions, not visual recognition.
        pair = next(reversed(current_report.bridge.pairs.values()))
        u,v = project_ground_point(pair['left']['camera_intrinsics'],pair['body_pose_enu'],point,20.)['pixel_uv']
        box=[max(0.,u/640-.06),max(0.,v/480-.07),min(1.,u/640+.06),min(1.,v/480+.07)]
    targets=[{'label':'蓝色油桶','bbox':box,'surroundings':[],'position_basis':'image_only'}]
    return tool_call('report_scene',{'capture_id':capture['capture_id'],'summary':'controlled target report',
        'targets':'>\n'+json.dumps(targets) if malformed else targets,'limitations':['controlled fixture']},
        'report-'+capture['capture_id']+('-bad' if malformed else ''))


def review(messages):
    value=json.loads(next(m for m in reversed(messages) if getattr(m,'name',None)=='report_scene').content)
    active=value['search_progress']['confirmation']
    return tool_call('review_target_candidate',{'current_target_id':active['current_target_ids'][0],
        'source_supported':True,'current_supported':True,'same_object':True,
        'source_description':'synthetic source','current_description':'synthetic current','reason':'controlled pair'},
        'review-'+active['current_capture_id'])


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.limits=TaskLimits(12,900.,60,1,30.,20.)

    def test_target_attribute_color_is_preserved_in_search_query_without_overwriting_raw_plan(self):
        value=proposal();value['targets'][0].update(label='油桶',attributes={'color':'blue'})
        plan=compile_plan(value,('t','c'),'找到蓝色油桶并报告，然后返航降落',self.limits)
        self.assertIn('蓝色',target_query(plan.targets[0]))
        self.assertEqual(plan.targets[0]['label'],'油桶')
        value['targets'][0]['label']='红色油桶'
        with self.assertRaises(ValueError):compile_plan(value,('t','c'),'找到蓝色油桶并报告',self.limits)

    def test_original_relative_goal_cannot_disappear_or_accept_model_coordinates(self):
        text='找到蓝色油桶并报告，降落在它前方'
        with self.assertRaises(ValueError):compile_plan(proposal(),('t','c'),text,self.limits)
        value=proposal(relative=True)
        plan=compile_plan(value,('t','c'),text,self.limits)
        self.assertEqual(plan.original_instruction,text)
        self.assertEqual(plan.stages[-1].relation,'object_front')
        value['stages'][-1]['position_enu_m']=[0.,0.,0.]
        with self.assertRaises(ValueError):compile_plan(value,('t','c'),text,self.limits)

    def test_close_photo_and_report_requirements_are_preserved(self):
        with self.assertRaises(ValueError):compile_plan(proposal(),('t','c'),'找到油桶，靠近拍照并报告',self.limits)
        value=proposal(close=True)
        value['stages']=[s for s in value['stages'] if s['kind']!='report']
        with self.assertRaises(ValueError):compile_plan(value,('t','c'),'找到油桶，靠近拍照并报告',self.limits)

    def test_unknown_target_and_terminal_stage_order_are_rejected(self):
        for value in (dict(proposal(), targets=[]),
                      dict(proposal(), stages=[{'kind':'close_photo','target_ref':'unknown'},{'kind':'hold'}]),
                      dict(proposal(), stages=[{'kind':'search','target_ref':'target'},{'kind':'hold'},{'kind':'report'}])):
            with self.assertRaises(ValueError):compile_plan(value,('t','c'),'找到油桶',self.limits)

    def test_overview_photo_cannot_be_silently_replaced_by_target_search(self):
        with self.assertRaises(ValueError):
            compile_plan(proposal(),('t','c'),'请为我拍摄一张俯瞰整个果园的照片',self.limits)

    def test_model_cannot_replace_required_home_end_with_hold(self):
        value=proposal();value['stages'][-1]={'kind':'hold'}
        with self.assertRaises(ValueError):
            compile_plan(value,('t','c'),'找到油桶并报告，然后返航降落',self.limits)

    def test_model_cannot_add_unrequested_object_relative_landing(self):
        with self.assertRaises(ValueError):
            compile_plan(proposal(relative=True),('t','c'),'找到油桶并报告',self.limits)


    def test_front_evidence_is_internal_without_invented_independent_route_requirements(self):
        text='寻找白色货车，取得有效前脸关系证据后，由程序选择车前落点；到位后用新图复核，再正常降落在货车前方。'
        constraints=plan_constraints(text)
        self.assertTrue(constraints['search_required'])
        self.assertTrue(constraints['relative_landing_required'])
        self.assertFalse(constraints['close_photo_required'])
        self.assertFalse(constraints['requested_route_actions']['approach'])
        self.assertIn('approach',constraints['unrequested_independent_route_actions'])
        explicit=plan_constraints('找到蓝色油桶，接近它并报告，然后返航降落')
        self.assertTrue(explicit['requested_route_actions']['approach'])
        self.assertNotIn('approach',explicit['unrequested_independent_route_actions'])

    def test_plan_request_schema_exposes_only_requested_actions_without_changing_execution_schema(self):
        from task_runtime.contracts import PlanProposal
        original=PlanProposal.model_json_schema();saved=copy.deepcopy(original)
        value=plan_request_schema(original,'寻找白色货车，取得前脸关系证据后，降落在货车前方')
        variants=value['$defs']['StageProposal']['anyOf']
        stages={s['properties']['kind']['enum'][0]:s for s in variants}
        self.assertEqual(list(stages),['search','relative_land'])
        self.assertNotIn('altitude_m',stages['search']['properties'])
        self.assertNotIn('relation',stages['search']['properties'])
        self.assertIn('relation',stages['relative_land']['required'])
        self.assertNotIn('null',str(stages['relative_land']['properties']['relation']))
        self.assertEqual(original,saved)
        close=plan_constraints('找到蓝色油桶，靠近拍照并报告，然后返航降落')
        self.assertIn('close_photo',close['allowed_stage_kinds'])
        self.assertNotIn('approach',close['allowed_stage_kinds'])
        photo=plan_request_schema(original,'拍摄当前区域，然后降落')
        photographic=next(s for s in photo['$defs']['StageProposal']['anyOf'] if s['properties']['kind']['enum']==['photo'])
        self.assertIn('altitude_m',photographic['properties'])


class StageAuthorityTests(unittest.TestCase):
    def setUp(self):
        self.fixture=authority_fixtures.AuthorityTests('test_valid_ticket_resolves_cached_goal_and_invalidates_all_alternatives')
        self.fixture.setUp()
        self.authority=self.fixture.authority

    def test_stage_transition_drops_tickets_preserves_scope_and_does_not_revoke_owner(self):
        before=self.authority.constraint()
        self.authority.advance_stage(self.fixture.context['owner'],'photo',1,self.fixture.identity)
        self.assertEqual(self.authority.tickets,{})
        with self.assertRaises(MissionError):self.fixture.consume()
        after=self.authority.constraint()
        for key in ('owner','identity','origin_xy_enu_m','search_radius_m'):
            self.assertEqual(before[key],after[key])
        self.assertNotIn(tuple(before['owner']),self.authority.revoked_owners)
        with self.assertRaises(MissionError):self.authority.advance_stage(before['owner'],'photo',1,self.fixture.identity)

    def test_operator_revoke_wrong_owner_or_new_epoch_cannot_transition(self):
        epoch=copy.deepcopy(self.fixture.identity)
        epoch['coordinate_epoch']['vio_epoch']+=1
        for owner,identity in ((['other','turn'],self.fixture.identity),(self.fixture.context['owner'],epoch)):
            with self.assertRaises(MissionError):self.authority.advance_stage(owner,'photo',1,identity)
        self.authority.revoke()
        with self.assertRaises(MissionError):
            self.authority.advance_stage(self.fixture.context['owner'],'photo',1,self.fixture.identity)


class ParentGuardTests(unittest.TestCase):
    def setUp(self):
        from search_tools import SearchManager
        self.bridge=RuntimeBridge()
        self.bridge.execute=lambda steps,description='':self.bridge.wait_for(
            self.bridge.request('/mission',{'steps':steps,'description':description}))
        self.manager=SearchManager(self.bridge)
        self.runtime=TaskOrchestrator(self.bridge,self.manager)
        self.owner=('parent-tests','actual-task')
        self.runtime.ensure(self.owner,'找到蓝色油桶并报告，然后返航降落')

    def test_model_response_revision_is_bound_at_request_time(self):
        self.runtime.before_model_call(self.owner,0)
        self.runtime.sessions[self.owner]['revision']=1
        with self.assertRaisesRegex(ValueError,'stage changed'):
            self.runtime.register_response(self.owner,[{'id':'late','name':'define_task_plan'}],0)
        self.assertEqual(self.runtime.sessions[self.owner]['issued_calls'],{})
        self.assertEqual(self.bridge.flight_requests,[])

    def test_search_reserves_observations_for_a_later_approach_and_its_fresh_verification(self):
        owner=('parent-tests','approach-task')
        self.runtime.ensure(owner,'找到蓝色油桶，接近它并报告，然后返航降落')
        value=proposal();value['stages'].insert(1,{'kind':'approach','target_ref':'target'})
        self.runtime.define(owner,value)
        child=self.manager.adaptive.sessions[owner]
        self.assertLessEqual(child['observation_ceiling'],self.runtime.sessions[owner]['limits'].max_observations-2)
        self.assertEqual(self.bridge.flight_requests,[])

    def test_tool_callback_can_only_be_consumed_once(self):
        from task_runtime.contracts import PlanProposal
        self.runtime.schemas={'define_task_plan':PlanProposal}
        call={'id':'one','name':'get_drone_state','args':{}}
        self.runtime.register_response(self.owner,[call],0)
        self.assertIsNone(self.runtime.guard_tool(self.owner,call))
        with self.assertRaises(ValueError):self.runtime.guard_tool(self.owner,call)
        self.assertEqual(self.bridge.flight_requests,[])

    def test_one_parent_correction_budget_is_shared_with_text_and_schema(self):
        from pydantic import BaseModel
        class Schema(BaseModel):
            targets:list
        self.runtime.schemas={'define_task_plan':Schema}
        self.runtime.correct_unsubmitted_text(self.owner)
        call={'id':'bad','name':'define_task_plan','args':{'targets':'not-a-list'}}
        self.runtime.register_response(self.owner,[call],0)
        with self.assertRaisesRegex(ValueError,'budget exhausted'):
            self.runtime.guard_tool(self.owner,call)
        self.assertEqual(self.runtime.sessions[self.owner]['recoveries'],1)
        self.assertEqual(self.bridge.flight_requests,[])

    def test_original_instruction_cannot_be_replaced_under_same_owner(self):
        with self.assertRaises(ValueError):self.runtime.ensure(self.owner,'改成另一个任务')

    def test_semantic_plan_rejection_is_unexecuted_and_shares_the_original_credit(self):
        invalid=proposal();invalid['stages'].insert(1,{'kind':'approach','target_ref':'target'})
        call={'id':'unrequested-route','name':'define_task_plan','args':{'plan':invalid}}
        self.runtime.register_response(self.owner,[call],0)
        result=self.runtime.guard_tool(self.owner,call)
        self.assertEqual(result['reason_code'],'plan_contract_invalid')
        self.assertFalse(result['tool_executed'])
        self.assertFalse(result['mission_submitted'])
        self.assertIn('approach',result['plan_constraints']['unrequested_independent_route_actions'])
        self.assertIsNone(self.runtime.sessions[self.owner]['plan'])
        self.assertNotIn(self.owner,self.manager.adaptive.sessions)
        self.assertEqual(self.bridge.flight_requests,[])
        self.assertEqual(self.runtime.sessions[self.owner]['recoveries'],1)
        with self.assertRaisesRegex(ValueError,'budget exhausted'):
            self.runtime.correct_unsubmitted_text(self.owner)

    def pending_observation(self, delivered=True):
        from single_vlm import FrameStore
        from test_vision import CONFIG
        self.runtime.define(self.owner, proposal())
        self.manager.adaptive.execute_pending(self.owner)
        store = self.runtime.frame_store = FrameStore(self.bridge, CONFIG, ttl_seconds=900, frames_per_turn=12)
        capture = store.capture(self.owner, 'controlled actual owned pixels')['capture_id']
        self.runtime.await_report(self.owner, capture)
        if delivered:
            store.mark_delivered(self.owner, capture, {})
            self.runtime.observation_delivered(self.owner, capture)
        return store, capture

    def redundant_observation(self, name='observe_target_search_view', identifier='redundant'):
        call = {'id': identifier, 'name': name, 'args': {}}
        self.runtime.register_response(self.owner, [call], self.runtime.sessions[self.owner]['revision'])
        return call

    def test_pending_capture_dominates_scheduling_and_needs_delivery_before_correction(self):
        store, capture = self.pending_observation(delivered=False)
        before = len(self.bridge.flight_requests)
        self.assertEqual(self.runtime.snapshot(self.owner)['next_tool'], 'report_scene')
        self.assertIsNone(self.runtime.program_call(self.owner))
        with self.assertRaisesRegex(RuntimeError, '尚未成功传给'):
            self.runtime.guard_tool(self.owner, self.redundant_observation())
        self.assertEqual(self.runtime.sessions[self.owner]['recoveries'], 0)
        store.mark_delivered(self.owner, capture, {})
        self.runtime.observation_delivered(self.owner, capture)
        call = self.redundant_observation(identifier='delivered')
        corrected = self.runtime.guard_tool(self.owner, call)
        self.assertEqual(corrected['capture_id'], capture)
        self.assertFalse(corrected['tool_executed'])
        self.assertFalse(corrected['mission_submitted'])
        with self.assertRaisesRegex(ValueError, 'Stale, duplicate'):
            self.runtime.guard_tool(self.owner, call)
        self.assertEqual(len(self.bridge.flight_requests), before)

    def test_observation_correction_cannot_bypass_live_or_image_identity_barriers(self):
        store, capture = self.pending_observation()
        call = self.redundant_observation()
        saved = copy.deepcopy(store.frames[capture])
        for fault in ('expired', 'owner', 'hash', 'revision', 'epoch'):
            with self.subTest(fault=fault):
                store.frames[capture] = copy.deepcopy(saved)
                store.clock = time.monotonic
                s = self.runtime.sessions[self.owner]
                revision = s['revision']
                if fault == 'expired':store.clock = lambda: saved['created_monotonic']+901
                elif fault == 'owner':store.frames[capture]['owner'] = ('other', 'turn')
                elif fault == 'hash':store.frames[capture]['camera']['image_sha256'] = 'a'*64
                elif fault == 'revision':s['revision'] += 1
                else:self.bridge.telemetry['external_vision']['adapter']['epoch'] += 1
                s['issued_calls'][call['id']] = (revision, call['name'])
                with self.assertRaises((ValueError, RuntimeError)):
                    self.runtime.guard_tool(self.owner, call)
                self.assertEqual(s['recoveries'], 0)
                s['revision'] = revision

    def test_pending_image_uses_shared_credit_and_other_disallowed_actions_stay_fatal(self):
        self.pending_observation()
        for name in ('execute_mission', 'capture_scene', 'land_drone'):
            with self.assertRaisesRegex(ValueError, 'outside the current parent stage'):
                self.runtime.guard_tool(self.owner, self.redundant_observation(name, name))
        self.assertEqual(self.runtime.sessions[self.owner]['recoveries'], 0)
        self.runtime.correct_unsubmitted_text(self.owner)
        with self.assertRaisesRegex(ValueError, 'budget exhausted'):
            self.runtime.guard_tool(self.owner, self.redundant_observation())
        self.assertEqual(self.runtime.sessions[self.owner]['recoveries'], 1)

    def test_new_capture_cannot_replace_an_unreported_image(self):
        store, capture = self.pending_observation()
        other = store.capture(self.owner, 'second controlled image')['capture_id']
        with self.assertRaisesRegex(ValueError, 'cannot be replaced'):
            self.runtime.await_report(self.owner, other)
        self.assertEqual(self.runtime.sessions[self.owner]['awaiting_report_capture_id'], capture)

    def test_staged_cache_keeps_original_pixels_for_the_existing_photo_and_time_budget(self):
        from single_vlm import build_single_vlm_agent
        from test_single_vlm import LocalScriptModel
        from test_vision import CONFIG
        with mock.patch.dict(os.environ, {'UAV_TASK_RUNTIME': 'staged'}):
            build_single_vlm_agent(self.bridge, CONFIG, [], '', model=LocalScriptModel(script=[]))
        store = self.bridge.task_runtime.frame_store
        now = [100.];store.clock = lambda: now[0]
        captures = []
        for index in range(MAX_OBSERVATIONS):
            identifier = store.capture(self.owner, 'controlled retained image')['capture_id']
            store.mark_delivered(self.owner, identifier, {})
            captures.append(identifier);now[0] += 50.
        self.assertEqual(store.get(self.owner, captures[0], require_delivered=True)['capture_id'], captures[0])
        self.assertEqual(len(store.frames), 16)
        self.assertEqual(store.ttl_seconds, self.runtime.sessions[self.owner]['limits'].max_duration_seconds)
        self.assertEqual(store.frames_per_turn, self.runtime.sessions[self.owner]['limits'].max_observations)
        with self.assertRaises(RuntimeError):store.get(('other', 'task'), captures[0])
        for index in range(4):store.capture(('other', 'task'), 'controlled other owner')
        self.assertLessEqual(len(store.frames), MAX_OBSERVATIONS+2)
        now[0] += 901.
        with self.assertRaises(RuntimeError):store.get(self.owner, captures[-1])

    def test_model_budget_charges_attempts_and_cannot_reset_with_a_stage(self):
        s=self.runtime.sessions[self.owner]
        from search_limits import MAX_OBSERVATIONS
        self.assertGreater(self.runtime.graph_step_limit(),2*(60+2*MAX_OBSERVATIONS))
        for _ in range(s['limits'].max_model_calls):self.runtime.before_model_call(self.owner,0)
        self.assertEqual(s['model_calls'],60)
        with self.assertRaises(ValueError):self.runtime.before_model_call(self.owner,0)
        self.assertEqual(s['model_calls'],60)

    def test_coordinate_reset_after_handoff_stays_a_safety_fault(self):
        # The actual graph/fixtures establish the original binding. No new
        # owner or repeated begin is used to clear a reset.
        client=self.bridge
        self.runtime.define(self.owner,proposal())
        self.manager.adaptive.execute_pending(self.owner)
        client.telemetry['external_vision']['adapter']['epoch']+=1
        with self.assertRaisesRegex(ValueError,'坐标系已重置'):self.runtime.require_live(self.owner)
        self.assertEqual(len(self.manager.adaptive.sessions),1)


class RuntimeGraphTests(unittest.TestCase):
    def test_attribute_color_reaches_actual_search_session_and_original_plan_stays_intact(self):
        value=proposal();value['targets'][0].update(label='油桶',attributes={'color':'blue'})
        result,app,_=self.run_graph([tool_call('define_task_plan',{'plan':value}),current_report],
                                  '找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        child=next(iter(app.audited_client.task_runtime.skills.search.sessions.values()))
        self.assertIn('蓝色',child['query'])
        self.assertEqual(result['task_runtime']['plan']['targets'][0]['label'],'油桶')

    def run_graph(self, script, prompt, bridge=None, confirmation='single_frame', timeout=15,
                  selector='deterministic', pool='ranked'):
        self.bridge=bridge or RuntimeBridge()
        fixture=nbv_fixtures.NBVGraphTests('test_multiview_graph_requires_paired_delivery_and_distinct_confirmation_move')
        with mock.patch.dict(os.environ,{'UAV_TASK_RUNTIME':'staged'}):
            return fixture.run_task(self.bridge,script,auto_prepare=True,prompt=prompt,confirmation=confirmation,timeout=timeout,
                                    selector=selector,pool=pool)

    def base(self, close=False, relative=False):
        return [tool_call('define_task_plan',{'plan':proposal(close,relative)},'plan')]

    def test_search_handoff_report_and_parent_finish_are_in_one_agent_graph(self):
        result,app,model=self.run_graph(self.base()+[current_report],
            '找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        parent=result['task_runtime_result']
        self.assertEqual(parent['goal_status'],'satisfied')
        self.assertEqual(len(parent['completed_stages']),3)
        self.assertTrue(result['task_outcome']['user_goal_completed'])
        events=result['task_runtime_events']
        handoff=next(v for v in events if v['event']=='stage_completed')
        self.assertIsNone(handoff['result'])
        self.assertEqual(handoff['plan']['stages'][1]['status'],'pending')
        runtime=app.audited_client.task_runtime
        session=next(iter(runtime.skills.search.sessions.values()))
        self.assertEqual(session['execution_mode'],'orchestrated')
        self.assertIn('skill_handoff',session)
        self.assertEqual(sum(r['steps'][0]['action']=='takeoff' for r in self.bridge.flight_requests if r['path']=='/mission'),1)
        self.assertEqual(len(result['model_calls']),2)  # plan / report; program schedules observation
        self.assertIn('controlled target report', result['reply'])
        self.assertIn(result['perceptions'][0]['capture_id'], result['reply'])
        self.assertTrue(any(v['event']=='program_local_grounding' for v in events))
        self.assertFalse(self.bridge.telemetry['armed'])

    def test_schema_invalid_report_corrects_once_without_new_photo_or_flight(self):
        script=self.base()+[lambda m:current_report(m,malformed=True),current_report]
        result,app,_=self.run_graph(script,'找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'],1)
        self.assertEqual(len(result['perceptions']),1)
        self.assertEqual(result['task_runtime']['budget']['observations_used'],1)
        self.assertIsNone(result['safety_landing'])

    def test_report_explicitly_negating_query_does_not_handoff_or_complete_search(self):
        label='灰色金属箱体（非蓝色油桶）'
        def negative(messages):
            call=current_report(messages)
            call.tool_calls[0]['args']['targets'][0]['label']=label
            return call
        result,app,_=self.run_graph(self.base()+[negative,RuntimeError('controlled end after negative report')],
            '找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['task_runtime']['target_memory'],{})
        self.assertFalse(any(e['event']=='stage_completed'for e in result['task_runtime_events']))
        self.assertEqual(result['perceptions'][0]['targets'][0]['label'],label)

    def test_staged_visual_nbv_selection_uses_owned_offered_ticket_and_normal_parent_end(self):
        from test_single_vlm import tool_payload
        reports=[0]
        def respond(messages):
            state=json.loads(next(m.content.split('父任务实际状态：',1)[1]for m in reversed(messages)
                if isinstance(getattr(m,'content',None),str)and m.content.startswith('父任务实际状态：')))
            name=state['next_tool']
            if name=='observe_target_search_view':return tool_call(name,{})
            if name=='report_scene':
                call=current_report(messages)
                if not reports[0]:call.tool_calls[0]['args']['targets']=[]
                reports[0]+=1
                return call
            if name=='select_target_search_view':
                progress=tool_payload(messages,'report_scene')['search_progress']
                self.assertGreaterEqual(len(progress['candidates']),2)
                self.assertNotIn('execution_ticket',json.dumps(progress['candidates']))
                return tool_call(name,{'batch_id':progress['batch_id'],'candidate_id':progress['candidates'][-1]['id'],
                                      'reason':'controlled choice among verified diverse views'})
            raise AssertionError('Unexpected model phase '+str(name))
        result,app,model=self.run_graph(self.base()+[respond]*12,'找到蓝色油桶并报告，然后返航降落',
            selector='vlm',pool='diverse',timeout=25)
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        self.assertEqual(self.bridge.ticket_executions,1)
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'],0)
        self.assertIsNone(result['safety_landing'])
        selection=next(c for c in result['model_calls']if c['image_purpose']=='nbv_candidate_selection')
        self.assertEqual(selection['tool_choice'],'select_target_search_view')

    def test_semantic_plan_error_corrects_before_any_child_or_flight_then_completes(self):
        invalid=proposal();invalid['stages'][0]['altitude_m']=3.
        script=[tool_call('define_task_plan',{'plan':invalid},'bad-plan')]+self.base()+[current_report]
        result,app,model=self.run_graph(script,'找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'],1)
        corrected=next(e for e in result['task_runtime_events']if e['event']=='unexecuted_plan_contract_correction')
        self.assertIsNone(corrected['plan'])
        self.assertEqual(corrected['budget']['observations_used'],0)
        self.assertIsNone(result['safety_landing'])
        self.assertEqual(model.bindings[0]['tool_choice'],'define_task_plan')
        self.assertEqual(model.bindings[1]['tool_choice'],'define_task_plan')

    def test_repeated_semantic_plan_error_terminates_on_ground_without_unbounded_retry(self):
        invalid=proposal();invalid['stages'].insert(1,{'kind':'approach','target_ref':'target'})
        script=[tool_call('define_task_plan',{'plan':invalid},'bad-plan-1'),
                tool_call('define_task_plan',{'plan':invalid},'bad-plan-2')]
        result,app,model=self.run_graph(script,'找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['error'],'Parent plan correction budget exhausted')
        self.assertEqual(len(model.calls),2)
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'],1)
        self.assertEqual(self.bridge.flight_requests,[])
        self.assertEqual(app.audited_client.task_runtime.skills.search.sessions,{})
        self.assertIsNone(result['safety_landing'])

    def test_second_schema_error_aborts_with_owned_landing(self):
        script=self.base()+[lambda m:current_report(m,malformed=True),lambda m:current_report(m,malformed=True)]
        result,app,_=self.run_graph(script,'找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'],1)
        self.assertNotEqual(result['task_runtime_result']['goal_status'],'satisfied')
        self.assertFalse(self.bridge.telemetry['armed'])
        self.assertEqual(self.bridge.telemetry['landed_state'],1)

    def test_invalid_bbox_corrects_same_delivered_photo_before_report_execution(self):
        def invalid_box(messages):
            call = current_report(messages)
            call.tool_calls[0]['args']['targets'][0]['bbox'] = [1.2, .2, .8, .9]
            return call
        result, app, _ = self.run_graph(self.base()+[invalid_box, current_report],
            '找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'], 'succeeded', result.get('error'))
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'], 1)
        self.assertEqual(result['task_runtime']['budget']['observations_used'], 1)
        self.assertEqual(len(result['perceptions']), 1)
        runtime = app.audited_client.task_runtime
        owner = next(iter(runtime.sessions))
        self.assertEqual(len([frame for frame in runtime.frame_store.frames.values()
                              if frame['owner'] == owner]), 1)
        self.assertIsNone(result['safety_landing'])

    def test_bbox_error_cannot_obtain_a_second_shared_format_correction(self):
        def reversed_box(messages):
            call = current_report(messages)
            call.tool_calls[0]['args']['targets'][0]['bbox'] = [.8, .2, .3, .9]
            return call
        result, _, _ = self.run_graph(self.base()+[
            lambda m: current_report(m, malformed=True), reversed_box],
            '找到蓝色油桶并报告，然后返航降落')
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['task_runtime']['budget']['recoveries_used'], 1)
        self.assertEqual(result['task_runtime']['budget']['observations_used'], 0)
        self.assertFalse(result['perceptions'])
        self.assertFalse(self.bridge.telemetry['armed'])
        self.assertEqual(self.bridge.telemetry['landed_state'], 1)

    def test_late_or_duplicate_model_tool_cannot_change_stage(self):
        result,app,_=self.run_graph(self.base()+[current_report],'找到蓝色油桶并报告，然后返航降落')
        runtime=app.audited_client.task_runtime
        owner=next(iter(runtime.sessions))
        count=len(self.bridge.flight_requests)
        with self.assertRaises(ValueError):runtime.guard_tool(owner,{'id':'first','name':'observe_target_search_view','args':{}})
        self.assertEqual(len(self.bridge.flight_requests),count)

    def test_search_then_close_photo_preserves_parent_map_epoch_budget_and_owner(self):
        bridge=RuntimeBridge();point=[10.,0.,1.]
        old=bridge.localize_stereo
        def localize(pair_id,boxes):
            data=old(pair_id,boxes);pair=bridge.pairs[pair_id]
            for item in data['targets']:
                snapshot={'body_pose_enu':pair['body_pose_enu'],'stamp_sim_s':pair['stamp_sim_s'],
                    'camera_intrinsics':pair['left']['camera_intrinsics'],
                    'stereo_image_sha256':pair['image_sha256'],'coordinate_epoch':item['coordinate_epoch']}
                item.update(measured_stereo({'snapshot':snapshot},point))
            return data
        bridge.localize_stereo=localize
        current_report.bridge=bridge
        report=lambda m:current_report(m,point)
        script=self.base(close=True)+[report]
        for index in range(8):
            script += [report, review]
        result,app,_=self.run_graph(script,'找到蓝色油桶，靠近拍照并报告，然后返航降落',bridge)
        self.assertEqual(result['status'],'succeeded',result.get('error'))
        self.assertTrue(result['task_outcome']['target_observation_completed'])
        runtime=app.audited_client.task_runtime;owner=next(iter(runtime.sessions));s=runtime.skills.search.sessions[owner]
        self.assertEqual(len(runtime.skills.search.semantic_maps.maps),1)
        self.assertEqual(len(s['observations']),result['task_runtime']['budget']['observations_used'])
        self.assertLessEqual(len(s['observations']),MAX_OBSERVATIONS)
        self.assertEqual(s['epoch'],runtime.skills.search.semantic_maps.maps[owner].coordinate_epoch)
        self.assertEqual(sum(r['steps'][0]['action']=='takeoff' for r in bridge.flight_requests if r['path']=='/mission'),1)

    def test_unimplemented_front_landing_remains_partial_after_valuable_search_and_report(self):
        bridge=RuntimeBridge()
        bridge.localize_stereo=lambda pair_id,boxes:nbv_fixtures.synthetic_metric_target(bridge,pair_id,boxes)
        current_report.bridge=bridge
        report=lambda m:current_report(m,[5.,0.,1.])
        script=self.base(relative=True)+[report,report,review]
        prompt='找到蓝色油桶并报告，降落在它前方'
        result,app,_=self.run_graph(script,prompt,bridge)
        self.assertEqual(result['task_contract']['admission'],'accepted')
        self.assertIn('task_runtime',result,result.get('error'))
        self.assertEqual(result['task_runtime']['plan']['original_instruction'],prompt)
        self.assertEqual(result['task_runtime_result']['goal_status'],'partial')
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['task_runtime_result']['unmet_requirements'][0]['kind'],'relative_land')
        self.assertIsNone(result['safety_landing'])  # Program recovery, not external success.
        self.assertFalse(bridge.telemetry['armed'])
        self.assertFalse(result['task_outcome']['user_goal_completed'])


if __name__=='__main__':unittest.main()
