"""Read-only visual search workbench embedded in the existing desktop."""
import math
import os
import time

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (QCheckBox, QGridLayout, QHBoxLayout, QLabel, QListWidget, QPlainTextEdit, QPushButton,
                               QSizePolicy, QSplitter, QTabWidget, QVBoxLayout, QWidget)
from desktop_search_data import (search_snapshot, observation_image, current_prediction, telemetry_matches,
                                 status_text, static_obstacle_outlines, landmark_uncertainty_ellipse)
from semantic_map import explicit_semantic_relation


def fmt(value, unit=''):
    return '{:.2f}{}'.format(value, unit) if type(value) in (int,float) and math.isfinite(value) else '—'


class ScaledImage(QLabel):
    """Keep the complete source image inside its current panel on every resize."""
    def __init__(self, text):
        super().__init__(text)
        self.original = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.setStyleSheet('background:#172332;color:#e2eaf1;')

    def set_image(self, pixmap, missing='暂无'):
        self.original = pixmap
        if pixmap is None or pixmap.isNull():
            self.setPixmap(QPixmap())
            self.setText(missing)
        else:
            self.rescale()

    def rescale(self):
        if self.original is not None and not self.original.isNull():
            self.setPixmap(self.original.scaled(self.contentsRect().size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.rescale()


class SearchMap(QWidget):
    def __init__(self):
        super().__init__(); self.setMinimumSize(260,260); self.snapshot=None; self.prediction=None
        self.candidate=None; self.selected_observation=None; self.position=None; self.trail=[];self.outlines=None;self.relations=[]
        self.layers={'coverage':True,'semantic':True,'observations':True,'candidates':True,'path':True,'predicted':False,'obstacles':True}
        self.zoom=1.;self.offset=QPointF();self.drag=None;self.follow=False

    def reset_view(self):
        self.zoom=1.;self.offset=QPointF();self.follow=False;self.update()

    def wheelEvent(self,event):
        self.zoom=max(.7,min(8.,self.zoom*(1.15 if event.angleDelta().y()>0 else 1/1.15)));self.update()

    def mousePressEvent(self,event):
        if event.button()==Qt.MouseButton.LeftButton:self.drag=event.position()

    def mouseMoveEvent(self,event):
        if self.drag is not None:
            self.offset += event.position()-self.drag;self.drag=event.position();self.update()

    def mouseReleaseEvent(self,event):
        self.drag=None

    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing);p.fillRect(self.rect(),QColor('#142332'))
        s=self.snapshot
        if not s:
            p.setPen(QColor('#d8e5ed'));p.drawText(self.rect(),Qt.AlignmentFlag.AlignCenter,'等待本任务有效空间地图');return
        grid=s['grid'];ox,oy=grid['origin_xy_enu_m'];span=grid['radius_m'];res=grid['resolution_m']
        scale=min(self.width(),self.height())/(2*span+3)*self.zoom
        center=QPointF(self.width()/2,self.height()/2)+self.offset
        cx,cy=(self.position[:2] if self.follow and self.position else (ox,oy))
        def point(x,y):return QPointF(center.x()+(x-cx)*scale,center.y()-(y-cy)*scale)
        eligible=set(s.get('eligible_cell_ids') or [])
        predicted=set((self.candidate or {}).get('visible_cell_ids') or []) if self.layers['predicted'] else set()
        for i in eligible:
            if type(i) is not int or not 0<=i<len(s['cells']):continue
            row,col=divmod(i,48); x=ox-span+col*res; y=oy-span+row*res; cell=s['cells'][i]
            rect=QRectF(point(x,y+res),point(x+res,y))
            color=QColor('#397bb0') if self.layers['coverage'] and cell.get('coverage',0)>0 else QColor('#405060')
            p.fillRect(rect,color)
            if self.layers['semantic'] and cell.get('semantic_score',0)>0:
                p.fillRect(rect,QColor(244,139,45,min(170,int(170*cell['semantic_score']))))
            if i in predicted and cell.get('coverage',0)==0:p.fillRect(rect,QColor(247,205,83,105))
        p.setBrush(Qt.BrushStyle.NoBrush);p.setPen(QPen(QColor('#b6c8d8'),1,Qt.PenStyle.DashLine))
        p.drawEllipse(point(ox,oy),span*scale,span*scale)
        if self.layers['obstacles'] and self.outlines:
            p.setBrush(Qt.BrushStyle.NoBrush);p.setPen(QPen(QColor('#d4d8db'),1))
            for outline in self.outlines:p.drawPolygon(QPolygonF([point(*xy) for xy in outline]))
        if len(self.trail)>1:
            p.setPen(QPen(QColor('#55e2c0'),2))
            for a,b in zip(self.trail,self.trail[1:]):p.drawLine(point(*a[:2]),point(*b[:2]))
        if self.layers['path'] and self.candidate:
            path=self.candidate.get('path') or {}; waypoints=[path.get('start_enu_m')]+(path.get('waypoints') or [])
            p.setPen(QPen(QColor('#f7ce66'),2,Qt.PenStyle.DashLine))
            for a,b in zip(waypoints,waypoints[1:]):
                if a and b:p.drawLine(point(*a[:2]),point(*b[:2]))
        if self.layers['candidates'] and self.prediction:
            items=(self.prediction.get('ranked_candidates') or [])[:5]
            if self.candidate and self.candidate not in items:items=items[:4]+[self.candidate]
            for c in items:
                pos=c.get('position_enu_m')
                if not pos or c.get('path_verified') is not True:continue
                p.setPen(QPen(QColor('#ffd66a') if self.candidate and c['id']==self.candidate.get('id') else QColor('#edf3f7'),1))
                p.drawRect(QRectF(point(*pos[:2])-QPointF(4,4),point(*pos[:2])+QPointF(4,4)));p.drawText(point(*pos[:2])+QPointF(6,-3),c['id'])
        if self.layers['observations']:
            for relation in self.relations:
                anchor,target=relation['anchor'],relation['target']
                p.setPen(QPen(QColor('#ed8dff'),2,Qt.PenStyle.DashLine))
                p.drawLine(point(*anchor[:2]),point(*target[:2]))
                middle=point((anchor[0]+target[0])/2,(anchor[1]+target[1])/2)
                p.drawText(middle,'指令：'+relation['relation'])
            selected=next((o for o in s.get('observations',[]) if o.get('capture_id')==self.selected_observation),None)
            if selected:
                p.setBrush(Qt.BrushStyle.NoBrush);p.setPen(QPen(QColor(145,230,255,120),1))
                for i in selected.get('visible_cell_ids',[]):
                    if type(i) is int and i in eligible:
                        row,col=divmod(i,48);x=ox-span+col*res;y=oy-span+row*res
                        p.drawRect(QRectF(point(x,y+res),point(x+res,y)))
            for index,o in enumerate(s.get('observations',[]),1):
                pos=o.get('position_enu_m')
                if pos:
                    p.setPen(QColor('#ffffff'));p.setBrush(QColor('#ffcf68') if o['capture_id']==self.selected_observation else QColor('#98c2e0'))
                    p.drawEllipse(point(*pos[:2]),4,4);p.drawText(point(*pos[:2])+QPointF(5,-5),str(index))
                    q=o.get('orientation_xyzw')
                    if isinstance(q,list) and len(q)==4 and all(type(v) in (int,float) and math.isfinite(v) for v in q):
                        x,y,z,w=q;heading=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                        p.drawLine(point(*pos[:2]),point(pos[0]+math.cos(heading),pos[1]+math.sin(heading)))
            for landmark in s.get('landmarks',[]):
                pos=landmark.get('position_enu_m');p.setPen(QColor('#ffab69'))
                if pos:
                    ellipse=landmark_uncertainty_ellipse(landmark.get('covariance_enu_m2'))
                    if ellipse:
                        major,minor,angle=ellipse;p.save();p.translate(point(*pos[:2]));p.rotate(-math.degrees(angle))
                        p.setBrush(Qt.BrushStyle.NoBrush);p.setPen(QPen(QColor('#ffab69'),1,Qt.PenStyle.DashLine))
                        p.drawEllipse(QRectF(-major*scale,-minor*scale,2*major*scale,2*minor*scale));p.restore()
                    p.setPen(QColor('#ffab69'));p.drawText(point(*pos[:2]),'◆ '+str(landmark.get('label',''))+' '+str(landmark.get('id','')))
                else:
                    for obs in (landmark.get('observations') or [])[-1:]:
                        origin=obs.get('bearing_origin_enu_m');ray=obs.get('bearing_direction_enu')
                        if origin and ray:
                            end=point(origin[0]+ray[0]*3,origin[1]+ray[1]*3);p.drawLine(point(*origin[:2]),end)
                            p.drawText(end,str(landmark.get('label',''))+'（方位，未定位）')
        if self.position:
            p.setBrush(QColor('#5ef1ce'));p.setPen(QColor('#142332'));p.drawEllipse(point(*self.position[:2]),6,6)
        p.setPen(QColor('#e2eaf1'));p.drawText(10,20,'ENU 坐标 · 米 | 蓝：几何观察 | 黄：预测 | 绿实线：采样航迹')
        p.drawText(10,40,'中心 X {} / Y {} · 观察点短线：机头方向'.format(fmt(cx),fmt(cy)))


class SearchWorkbench(QWidget):
    def __init__(self):
        super().__init__(); self.command=None;self.snapshot=None;self.prediction=None;self.evidence_dir=os.getenv('UAV_CAPTURE_EVIDENCE_DIR');self.selected_capture=None;self.selected_candidate=None;self.identity=None
        root=QVBoxLayout(self);self.status=QLabel('等待指令');self.status.setWordWrap(True);root.addWidget(self.status)
        self.flight_status=QLabel('飞行状态：等待遥测');root.addWidget(self.flight_status)
        self.metrics=QLabel('观察：— · 几何覆盖：— · 视点执行：— · 模型调用：—');self.metrics.setWordWrap(True);root.addWidget(self.metrics)
        split=QSplitter(Qt.Orientation.Horizontal);root.addWidget(split,1)
        left=QWidget();left_layout=QVBoxLayout(left);split.addWidget(left);self.map=SearchMap();left_layout.addWidget(self.map,1)
        controls=QGridLayout();left_layout.addLayout(controls)
        self.layer_checks={}
        for i,(key,text) in enumerate([('coverage','覆盖'),('semantic','语义'),('observations','观察/地标'),('candidates','候选'),('path','路径'),('predicted','预测视野'),('obstacles','静态轮廓')]):
            check=QCheckBox(text);check.setChecked(self.map.layers[key]);check.toggled.connect(lambda value,k=key:self.toggle_layer(k,value));controls.addWidget(check,i//4,i%4);self.layer_checks[key]=check
        reset=QPushButton('恢复全图');reset.clicked.connect(self.reset_map);controls.addWidget(reset,2,0,1,2)
        self.follow=QCheckBox('跟随');self.follow.toggled.connect(self.set_follow);controls.addWidget(self.follow,2,2)
        self.legend=QLabel('ENU 坐标，单位米 · 地图：等待 · 覆盖为几何可见性，不等于目标识别成功率');self.legend.setWordWrap(True);left_layout.addWidget(self.legend)
        right=QWidget();column=QVBoxLayout(right);split.addWidget(right);split.setSizes([700,430])
        column.addWidget(QLabel('下一视点（只读，路径经过预检查）'));self.candidates=QListWidget();self.candidates.setMaximumHeight(145);self.candidates.currentRowChanged.connect(self.choose_candidate);column.addWidget(self.candidates)
        self.decision=QLabel('暂无当前地图对应的候选');self.decision.setWordWrap(True);column.addWidget(self.decision)
        column.addWidget(QLabel('有效观察历史'));self.history=QListWidget();self.history.setMaximumHeight(150);self.history.currentRowChanged.connect(self.choose_observation);column.addWidget(self.history)
        tabs=self.detail_tabs=QTabWidget();column.addWidget(tabs,1)
        obs_tab=QWidget();obs_layout=QVBoxLayout(obs_tab);tabs.addTab(obs_tab,'观察照片')
        self.image=ScaledImage('选择观察以查看模型实际收到的照片');self.image.setMinimumSize(200,140);obs_layout.addWidget(self.image,1)
        self.report=QLabel('');self.report.setWordWrap(True);self.report.setTextFormat(Qt.TextFormat.PlainText);obs_layout.addWidget(self.report)
        pair_tab=QWidget();pair_layout=QVBoxLayout(pair_tab);tabs.addTab(pair_tab,'多视角复核')
        pair_split=QSplitter(Qt.Orientation.Vertical);pair_layout.addWidget(pair_split,1)
        pair_frame=QWidget();pair_images=QHBoxLayout(pair_frame);pair_split.addWidget(pair_frame);self.pair_images=[]
        for title in ('候选源图','当前观察'):
            cell=QVBoxLayout();cell.addWidget(QLabel(title));label=ScaledImage('暂无');label.setMinimumSize(90,75);cell.addWidget(label,1);pair_images.addLayout(cell);self.pair_images.append(label)
        self.pair_status=QPlainTextEdit('暂无多视角复核');self.pair_status.setReadOnly(True);self.pair_status.setMinimumHeight(95);pair_split.addWidget(self.pair_status);pair_split.setSizes([250,130])
        self.landmark_details=QPlainTextEdit();self.landmark_details.setReadOnly(True);tabs.addTab(self.landmark_details,'语义地标')
        self._reports=[];self._candidates=[];self._last_image=None;self._candidate_signature=None;self._pix_cache={};self.live_mission={}

    def toggle_layer(self,key,value):self.map.layers[key]=value;self.map.update()
    def set_follow(self,value):self.map.follow=value;self.map.update()
    def reset_map(self):self.follow.setChecked(False);self.map.reset_view()

    def update_data(self,state,command):
        old_id=(self.command or {}).get('id');self.command=command;self.status.setText(status_text(command,state))
        self.live_mission=state.get('mission') or {}
        flight=('已落地、上锁' if state.get('connected') is True and state.get('armed') is False and state.get('landed_state')==1 else
                '已解锁 / 未确认落地' if state.get('connected') is True and state.get('armed') is True else '等待可靠飞行遥测')
        self.flight_status.setText('飞行结果：'+flight)
        snap=search_snapshot(command,self.evidence_dir);self.snapshot=snap;self.map.snapshot=snap
        new_identity=(snap.get('world_sha256'),snap.get('sortie_id'),str(snap.get('coordinate_epoch'))) if snap else None
        if new_identity != self.identity or old_id != (command or {}).get('id'):
            self.map.trail=[];self.identity=new_identity;self.selected_capture=None;self.selected_candidate=None;self._pix_cache.clear()
            self.map.outlines=static_obstacle_outlines(snap,os.getenv('PX4_AGENT_WORLD'))
        if snap and telemetry_matches(state,snap):
            pos=state.get('position_enu_m')
            if isinstance(pos,(list,tuple)) and len(pos)>=3 and all(type(v) in (int,float) and math.isfinite(v) for v in pos[:3]):
                self.map.position=list(pos[:3]);self.map.trail=(self.map.trail+[list(pos[:2])])[-2000:] if not self.map.trail or math.dist(self.map.trail[-1],pos[:2])>.05 else self.map.trail
            else:self.map.position=None
        else:self.map.position=None;self.map.trail=[]
        self.prediction=current_prediction(command,snap);self.map.prediction=self.prediction
        self.map.relations=[]
        if snap and command:
            landmarks=snap.get('landmarks') or [];query=snap.get('query') or '';instruction=command.get('prompt') or ''
            targets=[x for x in landmarks if x.get('label')==query and x.get('position_enu_m')]
            for anchor in landmarks:
                label=anchor.get('label') or ''
                rel=explicit_semantic_relation(instruction,label,query) if label and query else None
                if rel and anchor.get('position_enu_m'):
                    for target in targets:
                        if target.get('id') != anchor.get('id'):
                            self.map.relations.append({'anchor':anchor['position_enu_m'],'target':target['position_enu_m'],'relation':rel})
        completed=(command or {}).get('flight_mission_results') or []
        executed={item.get('mission',{}).get('id') for item in completed if item.get('mission',{}).get('viewpoint_candidate_id') and item.get('mission',{}).get('status')=='succeeded' and item.get('mission',{}).get('id')}
        created=(command or {}).get('created_at');finished=(command or {}).get('finished_at')
        elapsed=(finished if type(finished) in (int,float) else time.time())-created if type(created) in (int,float) else None
        self.metrics.setText('有效观察 {} · 几何覆盖 {} · 已完成视点 {} · 模型调用 {} · 墙钟 {} s'.format(
            len(snap.get('observations',[])) if snap else '—',fmt(100*snap.get('geometric_coverage_fraction',0),'%') if snap else '—',
            len(executed) if command else '—',len((command or {}).get('model_calls') or []) if command else '—',fmt(elapsed)))
        validated=[c for c in ((self.prediction or {}).get('ranked_candidates') or []) if c.get('path_verified') is True]
        candidates=validated[:5]
        selected_id=(self.prediction or {}).get('selected_id')
        selected=next((c for c in validated if c.get('id')==selected_id),None)
        if selected and selected not in candidates:candidates=candidates[:4]+[selected]
        observations={o['capture_id']:o for o in snap.get('observations',[])} if snap else {}
        reports=[p for p in (command or {}).get('perceptions',[]) if p.get('status')=='ok'
                 and p.get('capture_id') in observations and (p.get('camera') or {}).get('image_sha256')
                 and (p.get('camera') or {}).get('image_sha256')==(observations[p['capture_id']].get('image_sha256') or {}).get('left')]
        signature=((self.prediction or {}).get('batch_id'),selected_id,tuple((c.get('id'),c.get('score')) for c in candidates))
        old_reports=[p.get('capture_id') for p in self._reports]
        self._candidates=candidates;self._reports=reports
        if signature != self._candidate_signature:
            self._candidate_signature=signature
            self.candidates.blockSignals(True);self.candidates.clear()
            for c in candidates:self.candidates.addItem('{} · {} · 覆盖 {} m²'.format(c.get('id'),c.get('direction',''),fmt(c.get('coverage_gain_m2'))))
            index=next((i for i,c in enumerate(candidates) if c['id']==(self.prediction or {}).get('selected_id')),0)
            if candidates:self.candidates.setCurrentRow(index);self.selected_candidate=candidates[index]['id']
            else:self.selected_candidate=None
            self.candidates.blockSignals(False)
        if old_reports != [p.get('capture_id') for p in reports]:
            self.history.blockSignals(True);self.history.clear()
            for i,report in enumerate(reports,1):self.history.addItem('观察 #{:02d} · {} · {} s'.format(i,report['capture_id'][:8],fmt(report.get('captured_at_sim_s'))))
            idx=next((i for i,r in enumerate(reports) if r['capture_id']==self.selected_capture),len(reports)-1)
            if idx>=0:self.history.setCurrentRow(idx);self.selected_capture=reports[idx]['capture_id']
            else:self.selected_capture=None
            self.history.blockSignals(False)
        self.refresh_selection()
        self.refresh_pair()
        self.refresh_landmarks()
        events=(command or {}).get('semantic_map_events') or []
        map_status=('有效' if snap else '已失效或不可用' if events and events[-1].get('status')!='valid' else '等待有效快照')
        self.legend.setText('ENU 坐标，单位米 · 地图：{} · 几何覆盖 {}（非识别成功率）\n白框：静态包围盒俯视投影 · 橙虚线：定位不确定范围，非物体占地{}'.format(
            map_status,fmt(100*snap.get('geometric_coverage_fraction',0),'%') if snap else '—',
            ('' if self.map.outlines is not None else ' · 静态轮廓不可用') + ('' if not snap or telemetry_matches(state,snap) else ' · 实时坐标身份不匹配，已隐藏飞机')))
        self.map.update()

    def choose_candidate(self,index):
        self.selected_candidate=self._candidates[index]['id'] if 0<=index<len(self._candidates) else None;self.refresh_selection();self.map.update()
    def choose_observation(self,index):
        self.selected_capture=self._reports[index]['capture_id'] if 0<=index<len(self._reports) else None;self.refresh_selection();self.map.update()

    def refresh_selection(self):
        c=next((x for x in self._candidates if x.get('id')==self.selected_candidate),None);self.map.candidate=c;self.map.selected_observation=self.selected_capture
        if c:
            mode=(self.prediction or {}).get('planning_mode');selected=(self.prediction or {}).get('selected_id')==c['id']
            mission=self.live_mission;batch=(mission.get('search_constraint') or {}).get('batch_id')
            stage=('执行器'+{'running':'执行中','succeeded':'已完成','failed':'失败','cancelled':'已取消'}.get(mission.get('status'),'已接受')
                   if selected and mission.get('viewpoint_candidate_id')==c['id'] and batch==(self.prediction or {}).get('batch_id') else '已选择，等待执行器状态' if selected else '候选预览')
            labels={'coverage':'覆盖','semantic':'语义','motion':'移动代价','repeat':'重复代价','target_visibility':'目标可见性','parallax':'视差'}
            selector='模型选择：'+str((self.prediction or {}).get('selection_reason','')) if (self.prediction or {}).get('selector')=='vlm' else '确定性评分选择'
            self.decision.setText('{} · {} · {}\n新增覆盖 {} m² · 语义 {} · 路径 {} m · 预计 {} s\n评分贡献：{}\n{}'.format(
                '确认视角' if mode=='confirmation' else '探索视角',stage,c.get('direction',''),
                fmt(c.get('coverage_gain_m2')),fmt(c.get('semantic_gain_m2')),fmt((c.get('path') or {}).get('path_length_m')),fmt(c.get('motion_seconds')),
                ' / '.join('{} {}'.format(labels.get(k,k),fmt(v)) for k,v in (c.get('score_terms') or {}).items()),selector))
        else:self.decision.setText('暂无当前地图对应的候选；预览不代表已执行。')
        report=next((x for x in self._reports if x.get('capture_id')==self.selected_capture),None)
        if not report:self.image.set_image(None,'暂无已保存的有效观察');self.report.clear();return
        pix=self.report_pixmap(report)
        if pix is None:self.image.set_image(None,'原图缺失或哈希不符');self.report.setText(report.get('summary',''));return
        self._last_image=pix;self.image.set_image(pix)
        self.report.setText('截图 {} · 仿真时间 {} s\n{}'.format(self.selected_capture[:12],fmt(report.get('captured_at_sim_s')),report.get('summary','')))

    def report_pixmap(self,report):
        capture=report.get('capture_id');signature=None
        try:
            path=os.path.join(self.evidence_dir,capture+'.jpg');stat=os.stat(path)
            signature=(capture,stat.st_mtime_ns,stat.st_size,str(report.get('targets')),str(report.get('camera')))
            if signature in self._pix_cache:return self._pix_cache[signature]
        except (OSError,TypeError):pass
        data=observation_image(self.command,report.get('capture_id'),self.evidence_dir)
        pix=QPixmap()
        if not data or not pix.loadFromData(data):return None
        painter=QPainter(pix);painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        font=QFont();font.setPixelSize(17);painter.setFont(font)
        for target in report.get('targets',[]):
            box=target.get('bbox');
            if not isinstance(box,list) or len(box)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in box):continue
            x1,y1,x2,y2=box
            if not (0<=x1<x2<=1 and 0<=y1<y2<=1):continue
            painter.setPen(QPen(QColor('#ffcf5b'),3));painter.drawRect(QRectF(x1*pix.width(),y1*pix.height(),(x2-x1)*pix.width(),(y2-y1)*pix.height()))
            label=str(target.get('label','目标'))
            if label == report.get('query'):label+=' · 疑似目标'
            if (target.get('stereo_location') or {}).get('status') != 'ok':label+=' · 无可靠深度'
            painter.drawText(QPointF(x1*pix.width()+3,max(15,y1*pix.height()-4)),label)
        painter.end()
        if signature is not None:
            self._pix_cache[signature]=pix
            if len(self._pix_cache)>16:self._pix_cache.pop(next(iter(self._pix_cache)))
        return pix

    def refresh_pair(self):
        events=(self.command or {}).get('target_confirmation_events') or []
        event=next((e for e in reversed(events) if (e.get('candidate') or {}).get('current_capture_id')),None)
        candidate=(event or {}).get('candidate') or {}
        for label,key in zip(self.pair_images,('source_capture_id','current_capture_id')):
            capture=candidate.get(key);report=next((r for r in self._reports if r.get('capture_id')==capture),None)
            pix=self.report_pixmap(report) if report else None
            label.set_image(pix,'图片不可用')
        if not event:
            if self.pair_status.toPlainText() != '暂无多视角复核':self.pair_status.setPlainText('暂无多视角复核')
            return
        geometry=event.get('geometry') or {};outcome={'confirmed':'已确认','unconfirmed':'未确认','rejected':'已否定候选','review_required':'等待复核'}.get(event.get('outcome'),str(event.get('outcome','等待')))
        def support(side):
            if event.get(side+'_uncertain') is True:return '不确定'
            return {True:'支持',False:'不支持'}.get(event.get(side+'_supported'),'未给出')
        basis=geometry.get('association_basis',geometry.get('basis'))
        basis_text={'stereo_unique':'双目唯一关联','single_depth_projection':'单侧深度投影','bearing_intersection':'中心视线相容（仅假设）',
                    'clipped_region_hypothesis':'裁切区域相容（仅假设）','shared_stereo_target_memory':'双目目标记忆',
                    'one_metric_surface_projects_to_other_image':'单侧深度投影','bearing_intersection_not_localization':'中心视线相容',
                    'clipped_box_epipolar_region_not_localization':'裁切区域相容（仅假设）'}.get(basis,str(basis or '未给出'))
        text='{}\n源图支持：{} · 当前图支持：{}\n几何相容：{}\n关联依据：{}'.format(outcome,
            support('source'),support('current'),{True:'相容',False:'不相容'}.get(geometry.get('compatible'),'未给出'),basis_text)
        if self.pair_status.toPlainText()!=text:self.pair_status.setPlainText(text)

    def refresh_landmarks(self):
        snap=self.snapshot
        if not snap:
            text='暂无有效地图或语义地标。'
        else:
            landmarks=snap.get('landmarks') or []
            lines=['地标与候选线索：{} 条'.format(len(landmarks)),
                   '语义热区：'+('已由任务关系激活（不是概率）' if snap.get('semantic_field_active') else '未激活'),
                   '图像框为识别区域；地面物体占据范围尚未测得。椭圆是未标定的位置不确定范围，不代表物体大小。']
            relations=[]
            for item in landmarks:
                pos=item.get('position_enu_m');observations=item.get('observations') or []
                location=('ENU {}, {}, {} m（双目可见表面估计）'.format(*(fmt(v) for v in pos[:3]))
                          if isinstance(pos,list) and len(pos)>=3 else '未定位：仅有图像框与方位线索')
                status={'unlocalized':'未定位','candidate':'候选','confirmed':'多视点几何证据'}.get(item.get('status'),'未知')
                last=observations[-1] if observations else {};box=last.get('bbox')
                if isinstance(box,list) and len(box)==4 and all(type(v) in (int,float) and math.isfinite(v) for v in box):
                    area=max(0.,box[2]-box[0])*max(0.,box[3]-box[1])*100
                    region='图像框：[{:.2f}, {:.2f}, {:.2f}, {:.2f}]；约占画面 {:.1f}%'.format(*box,area)
                else:region='图像框：不可用'
                lines.append('{} · {} · {} · {} 次报告\n{}\n{}'.format(item.get('label','未命名'),item.get('id',''),status,len(observations),location,region))
                label=item.get('label') or '';query=snap.get('query') or ''
                relation=explicit_semantic_relation((self.command or {}).get('prompt') or '',label,query) if label and query else None
                if relation:
                    relations.append('指令描述：{} {} {}（不是视觉关系确认）{}'.format(label,relation,query,
                        '；地标未定位，未产生空间热区' if pos is None else ''))
            lines.extend(relations)
            if not relations:lines.append('当前未匹配到已观察地标与目标的明确任务关系；不会推测。')
            if not landmarks:lines.append('尚无已报告的视觉地标。')
            text='\n\n'.join(lines)
        if self.landmark_details.toPlainText()!=text:self.landmark_details.setPlainText(text)
