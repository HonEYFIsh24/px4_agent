"""Explicitly enabled desktop acceptance recorder; never runs in normal startup.

The human/driver must create start.json after ground and provider preflight. It
uses the real desktop input and submission method and records actual UI frames;
it never constructs observations, candidates, flight results, or model output.
"""
import json
import os
import time
from pathlib import Path

from PySide6.QtCore import QTimer


class WorkbenchAcceptance:
    def __init__(self, window, directory):
        self.window=window;self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        if (self.directory/'result.json').exists() or (self.directory/'start.json').exists():
            raise ValueError('Refusing to reuse desktop acceptance directory')
        self.records=[];self.marked=set();self.started=False;self.completed=False;self.start_mono=None
        self.timer=QTimer(window.root);self.timer.timeout.connect(self.tick);self.timer.start(250)
        (self.directory/'ready.json').write_text(json.dumps({'status':'waiting_for_explicit_start','pid':os.getpid()}))

    def save(self, command, state):
        record={'command':command,'flight':{k:state.get(k) for k in ('ready','connected','armed','landed_state','position_enu_m','external_vision','navigation')},
                'search_mode':os.getenv('UAV_SEARCH_MODE','fixed'),'target_confirmation_mode':os.getenv('UAV_TARGET_CONFIRMATION','single_frame'),
                'nbv_selector':os.getenv('UAV_NBV_SELECTOR','deterministic'),'candidate_pool_mode':os.getenv('UAV_NBV_CANDIDATE_POOL','ranked'),
                'heading_mode':os.getenv('UAV_NBV_HEADING_MODE','fixed'),'nbv_auto_prepare':os.getenv('UAV_NBV_AUTO_PREPARE','0')=='1',
                'capture_directory':os.getenv('UAV_CAPTURE_EVIDENCE_DIR'),'ui_events':self.records}
        path=self.directory/'result.tmp';path.write_text(json.dumps(record,ensure_ascii=False,indent=2));path.replace(self.directory/'result.json')

    def mark(self,name,command,state):
        if name in self.marked:return
        self.marked.add(name);w=self.window;wb=w.workbench
        image=name+'.png';ok=w.root.grab().save(str(self.directory/image),'PNG')
        self.records.append({'event':name,'wall_s':time.time(),'screenshot':image if ok else None,
            'workbench_visible':w.notebook.currentWidget() is wb and wb.isVisible(),
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
                    or state.get('armed') is not False or state.get('landed_state')!=1
                    or (state.get('external_vision') or {}).get('used_for_flight_control') is not True
                    or (state.get('navigation') or {}).get('method')!='static_3d'
                    or os.getenv('UAV_SEARCH_MODE')!='nbv_geometry'):
                raise ValueError('Explicit start or grounded VIO/static-NBV precondition failed')
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
        if wb.prediction and len(wb._candidates)>=2 and wb.selected_candidate and wb.map.candidate:
            self.mark('05-nbv-preview',command,state)
        if mission.get('status')=='running' and mission.get('viewpoint_candidate_id'):
            self.mark('06-viewpoint-moving',command,state)
        if wb.snapshot and len(wb._reports)>=2:self.mark('07-second-observation',command,state)
        if any(e.get('outcome') in ('confirmed','unconfirmed','rejected') for e in command.get('target_confirmation_events',[])):
            self.mark('08-multiview-review',command,state)
        if mission.get('status')=='running' and mission.get('action') in ('return_home','land'):self.mark('09-return-or-land',command,state)
        if command.get('status')!='running' and state.get('connected') is True and state.get('armed') is False and state.get('landed_state')==1:
            self.mark('10-terminal-grounded',command,state);self.completed=True
        self.save(command,state)
