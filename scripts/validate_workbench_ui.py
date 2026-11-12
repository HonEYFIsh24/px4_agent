"""Explicitly enabled desktop acceptance recorder; never runs in normal startup.

The human/driver must create start.json after ground and provider preflight. It
uses the real desktop input and submission method and records actual UI frames;
it never constructs observations, candidates, flight results, or model output.
"""
import json
import os
import time
import hashlib
import math
from pathlib import Path

from PySide6.QtCore import QTimer


def observer_start_scope(request, state, directory, run):
    """Record an explicit controlled start, without granting flight authority."""
    if 'controlled_observer_setup' not in request:
        if state.get('armed') is not False or state.get('landed_state') != 1:
            raise ValueError('Desktop acceptance requires a grounded start or explicit controlled observer setup')
        return {'mode': 'grounded'}
    name = request['controlled_observer_setup']
    if name != 'controlled-observer-setup.json':
        raise ValueError('Invalid controlled observer setup reference')
    path = Path(directory).parent/name
    raw = path.read_bytes();setup = json.loads(raw)
    previous = setup.get('final_state') or {}
    mission = state.get('mission') or {};expected = previous.get('mission') or {}
    external = state.get('external_vision') or {}
    navigation = state.get('navigation') or {};old_navigation = previous.get('navigation') or {}
    from search_tools import coordinate_epoch
    epoch = coordinate_epoch(state)
    if (setup.get('status') != 'succeeded' or setup.get('run') != run
            or setup.get('target_truth_queried') is not False or setup.get('not_counted_as_goal_pose') is not True
            or state.get('armed') is not True or state.get('landed_state') != 2
            or not mission.get('id') or mission['id'] != expected.get('id') or mission.get('status') != 'succeeded'
            or expected.get('status') != 'succeeded' or external.get('healthy') is not True
            or not navigation.get('registration_id') or navigation['registration_id'] != old_navigation.get('registration_id')
            or not epoch or epoch.get('vio_epoch') is None or epoch.get('ekf_counters') is None
            or epoch != coordinate_epoch(previous)
            or not (state.get('scene') or {}).get('world_sha256')
            or (state.get('scene') or {}).get('world_sha256') != (previous.get('scene') or {}).get('world_sha256')
            or math.dist(state['position_enu_m'], previous['position_enu_m']) > .4):
        raise ValueError('Controlled observer setup no longer matches the actual mission, pose or coordinate identity')
    return {'mode': 'controlled_airborne', 'scope': 'front relation skill; not broad-yard search',
            'setup_file': name, 'setup_sha256': hashlib.sha256(raw).hexdigest(),
            'setup_mission_id': mission['id'], 'setup_is_goal_pose_evidence': False}


class WorkbenchAcceptance:
    def __init__(self, window, directory):
        self.window=window;self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        if (self.directory/'result.json').exists() or (self.directory/'start.json').exists():
            raise ValueError('Refusing to reuse desktop acceptance directory')
        self.records=[];self.marked=set();self.started=False;self.completed=False;self.start_mono=None
        self.seed_evidence=None;self.world_file=None;self.run=None;self.observer_start=None
        self.timer=QTimer(window.root);self.timer.timeout.connect(self.tick);self.timer.start(250)
        (self.directory/'ready.json').write_text(json.dumps({'status':'waiting_for_explicit_start','pid':os.getpid()}))

    def save(self, command, state):
        record={'command':command,'flight':{k:state.get(k) for k in ('ready','connected','armed','landed_state','position_enu_m','external_vision','navigation')},
                'search_mode':os.getenv('UAV_SEARCH_MODE','fixed'),'target_confirmation_mode':os.getenv('UAV_TARGET_CONFIRMATION','single_frame'),
                'nbv_selector':os.getenv('UAV_NBV_SELECTOR','deterministic'),'candidate_pool_mode':os.getenv('UAV_NBV_CANDIDATE_POOL','ranked'),
                'heading_mode':os.getenv('UAV_NBV_HEADING_MODE','fixed'),'nbv_auto_prepare':os.getenv('UAV_NBV_AUTO_PREPARE','0')=='1',
                'capture_directory':os.getenv('UAV_CAPTURE_EVIDENCE_DIR'),'ui_events':self.records,
                'run':self.run,'world_file':self.world_file,'gazebo_seed_evidence':self.seed_evidence,
                'initial_observer_setup':self.observer_start}
        path=self.directory/'result.tmp';path.write_text(json.dumps(record,ensure_ascii=False,indent=2));path.replace(self.directory/'result.json')

    def mark(self,name,command,state):
        if name in self.marked:return
        self.marked.add(name);w=self.window;wb=w.workbench
        image=name+'.png';ok=w.root.grab().save(str(self.directory/image),'PNG')
        self.records.append({'event':name,'wall_s':time.time(),'screenshot':image if ok else None,
            'workbench_visible':w.notebook.currentWidget() is w.workbench_page and wb.isVisible(),
            'window_active':w.root.isActiveWindow(),'window_visible':w.root.isVisible(),
            'detail_tab_index':wb.detail_tabs.currentIndex(),
            'command_id':command.get('id') if command else None,'status_text':wb.status.text(),
            'flight_text':wb.flight_status.text(),'metrics_text':wb.metrics.text(),'legend_text':wb.legend.text(),
            'selected_capture':wb.selected_capture,'observation_count':len(wb._reports),
            'candidate_count':len(wb._candidates),'selected_candidate':wb.selected_candidate,
            'predicted_layer':wb.map.layers.get('predicted'),'candidate_has_path':bool((wb.map.candidate or {}).get('path')),
            'position':wb.map.position,'sampled_trail_points':len(wb.map.trail),
            'actual_mission':state.get('mission')})

    def tick(self):
        try:self._tick()
        except Exception as exc:
            # Do not terminate the Agent or disguise a failed recorder as a flight result.
            (self.directory/'recorder-error.txt').write_text(type(exc).__name__+': '+str(exc))
            self.timer.stop()

    def _tick(self):
        w=self.window;state=w.state or {};trigger=self.directory/'start.json'
        if not self.started:
            if not trigger.is_file():return
            request=json.loads(trigger.read_text());prompt=request.get('prompt')
            if (request.get('authorized') is not True or not isinstance(prompt,str) or not prompt.strip()
                    or state.get('simulation_only') is not True or state.get('ready') is not True
                    or time.monotonic()-w.last_state_at >= 2
                    or (state.get('external_vision') or {}).get('used_for_flight_control') is not True
                    or (state.get('navigation') or {}).get('method')!='static_3d'
                    or os.getenv('UAV_SEARCH_MODE')!='nbv_geometry'):
                raise ValueError('Explicit start or grounded VIO/static-NBV precondition failed')
            self.run=Path(os.environ['UAV_CAPTURE_EVIDENCE_DIR']).resolve().parent.name
            self.observer_start=observer_start_scope(request,state,self.directory,self.run)
            world=Path(os.environ['PX4_AGENT_WORLD']).resolve()
            seed=int(os.environ['UAV_WORKBENCH_ACCEPTANCE_GAZEBO_SEED'])
            from scripts.validate_autonomous_search import gazebo_seed_evidence
            self.seed_evidence=gazebo_seed_evidence(seed,world)
            if hashlib.sha256(world.read_bytes()).hexdigest() != (state.get('scene') or {}).get('world_sha256'):
                raise ValueError('Desktop scene does not match actual world file')
            self.world_file=str(world)
            self.started=True;self.start_mono=time.monotonic();w.notebook.setCurrentIndex(w.workbench_tab)
            w.workbench.layer_checks['predicted'].setChecked(True)
            w.prompt.setPlainText(prompt);self.mark('01-input',None,state);w.submit();return
        if self.completed:return
        command=w.application.command(w.command_id) if w.command_id else None
        if not command:return
        wb=w.workbench;mission=state.get('mission') or {};events=command.get('nbv_events') or []
        self.mark('02-submitted',command,state)
        if mission.get('action')=='takeoff' and mission.get('status')=='running':self.mark('03-takeoff',command,state)
        if wb.snapshot and len(wb._reports)>=1:self.mark('04-first-observation',command,state)
        # Capture the actual live tab once while retaining the operator's tab.
        # The live widget is populated only by the desktop freshness gate.
        if (wb.snapshot and wb.live_image.original is not None and '04-live-camera' not in self.marked
                and wb.detail_tabs.currentIndex() != 1):
            previous=wb.detail_tabs.currentIndex()
            wb.detail_tabs.setCurrentWidget(wb.live_image.parentWidget())
            self.mark('04-live-camera',command,state)
            wb.detail_tabs.setCurrentIndex(previous)
        if (wb.prediction and len(wb._candidates)>=2 and wb.selected_candidate==wb.prediction.get('selected_id')
                and wb.map.candidate and wb.map.layers.get('predicted')):
            self.mark('05-nbv-preview',command,state)
        if (mission.get('status')=='running' and mission.get('viewpoint_candidate_id')
                and wb.prediction and wb.selected_candidate==mission.get('viewpoint_candidate_id')
                and (mission.get('search_constraint') or {}).get('batch_id')==wb.prediction.get('batch_id')):
            self.mark('06-viewpoint-moving',command,state)
        if (wb.snapshot and len(wb._reports)>=2 and wb.selected_capture==wb._reports[-1].get('capture_id')
                and wb.detail_tabs.currentIndex()==0):self.mark('07-second-observation',command,state)
        review_labels={'confirmed':'已确认','unconfirmed':'未确认','rejected':'已否定候选'}
        if (wb.detail_tabs.currentIndex()==1 and any(e.get('outcome') in review_labels
                and (e.get('candidate',{}).get('source_capture_id'), e.get('candidate',{}).get('current_capture_id'))==wb._shown_review_pair
                and wb.pair_status.toPlainText().split('\n',1)[0]==review_labels[e['outcome']]
                for e in command.get('target_confirmation_events',[]))):
            self.mark('08-multiview-review',command,state)
        if mission.get('status')=='running' and mission.get('action') in ('return_home','land'):self.mark('09-return-or-land',command,state)
        if (command.get('status')!='running' and (wb.command or {}).get('status')==command.get('status')
                and time.monotonic()-w.last_state_at < 2 and state.get('connected') is True
                and state.get('armed') is False and state.get('landed_state')==1):
            self.mark('10-terminal-grounded',command,state);self.completed=True
        self.save(command,state)
