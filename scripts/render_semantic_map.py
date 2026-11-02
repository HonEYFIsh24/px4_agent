#!/usr/bin/env python3
"""Render an offline map snapshot with system matplotlib; no model or flight IO."""
import argparse
import json
import math
from pathlib import Path


def render(snapshot, output, world=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse, Polygon
    import numpy as np
    from matplotlib import font_manager
    chinese_font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if chinese_font.exists():
        font_manager.fontManager.ttflist.insert(0, font_manager.FontEntry(
            fname=str(chinese_font), name='PX4 Chinese'))
        plt.rcParams['font.family'] = 'PX4 Chinese'
    else:
        plt.rcParams['font.family'] = ['DejaVu Sans', 'Noto Sans CJK SC']
    plt.rcParams['axes.unicode_minus'] = False
    grid = snapshot['grid']
    x,y = grid['origin_xy_enu_m']
    extent = [x-12,x+12,y-12,y+12]
    eligible = set(snapshot['eligible_cell_ids'])
    coverage = np.full(2304, -1.)
    semantic = np.full(2304, np.nan)
    for i in eligible:
        coverage[i] = snapshot['cells'][i]['coverage']
        semantic[i] = snapshot['cells'][i]['semantic_score']
    fig, axes = plt.subplots(1,2,figsize=(13,6),sharex=True,sharey=True)
    from matplotlib.colors import ListedColormap, BoundaryNorm
    cmap = ListedColormap(['#555b61','#dce0e4','#438dc4'])
    for ax in axes:
        ax.imshow(coverage.reshape(48,48), origin='lower',extent=extent,cmap=cmap,
                  norm=BoundaryNorm([-1.5,-.5,.5,1.5],3),interpolation='none')
        ax.set_aspect('equal');ax.set_xlim(extent[:2]);ax.set_ylim(extent[2:])
        ax.set_xlabel('本地 ENU x / m');ax.grid(alpha=.18)
    axes[0].set_ylabel('本地 ENU y / m')
    heat = axes[1].imshow(np.ma.masked_less(semantic.reshape(48,48), .05),origin='lower',extent=extent,
                         cmap='YlOrRd',vmin=0,vmax=1,alpha=.72,interpolation='none')
    fig.colorbar(heat, ax=axes[1],fraction=.04,pad=.03,label='语义吸引分数（非概率）')
    # Geometry is unlabeled collision data, never semantic truth.
    boxes = snapshot.get('debug_raw_boxes', [])
    if world is not None:
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from static_navigation import collision_boxes
        boxes = collision_boxes(world, snapshot['world_sha256'])
    reg = snapshot['registration'];c,s = math.cos(reg['world_from_enu_yaw_rad']),math.sin(reg['world_from_enu_yaw_rad'])
    ox,oy,_ = reg['world_from_enu_origin_m']
    for low, high in boxes:
        corners=[]
        for a,b in ((low[0],low[1]),(high[0],low[1]),(high[0],high[1]),(low[0],high[1])):
            corners.append([c*(a-ox)+s*(b-oy),-s*(a-ox)+c*(b-oy)])
        for ax in axes:ax.add_patch(Polygon(corners,closed=True,fill=False,edgecolor='#30343b',linewidth=.6))
    for path in snapshot.get('recorded_navigation_paths', []):
        points=path['points_enu_m']
        for ax in axes:ax.plot([p[0] for p in points],[p[1] for p in points],color='#226b42',linewidth=1,alpha=.7)
    for number, observation in enumerate(snapshot['observations'],1):
        p=observation['position_enu_m']
        for ax in axes:
            ax.plot(p[0],p[1],'ko',markersize=3)
            ax.text(p[0]+.15,p[1]+.15,str(number),fontsize=8)
    for item in snapshot['landmarks']:
        if item['position_enu_m'] is None:
            for evidence in item['observations']:
                p,d=evidence['bearing_origin_enu_m'],evidence['bearing_direction_enu']
                for ax in axes:ax.arrow(p[0],p[1],d[0]*3,d[1]*3,width=.015,color='#b16c24',alpha=.45,length_includes_head=True)
            continue
        p=item['position_enu_m'];cov=np.array(item['covariance_enu_m2'])[:2,:2]
        values,vectors=np.linalg.eigh(cov);order=values.argsort()[::-1];values,vectors=values[order],vectors[:,order]
        angle=math.degrees(math.atan2(vectors[1,0],vectors[0,0]))
        for ax in axes:
            ax.plot(p[0],p[1],'x',color='#a32828',markersize=7)
            ax.text(p[0]+.2,p[1],item['label']+'\n'+item['status'],fontsize=7,color='#712525')
            ax.add_patch(Ellipse(p[:2],6*math.sqrt(max(values[0],0)),6*math.sqrt(max(values[1],0)),
                                 angle=angle,fill=False,edgecolor='#a32828',alpha=.7))
    prediction = snapshot.get('viewpoint_prediction')
    if prediction:
        for rank, candidate in enumerate(prediction['ranked_candidates']):
            p = candidate['position_enu_m']
            chosen = candidate['id'] == prediction['selected_id']
            for ax in axes:
                ax.plot(p[0],p[1],marker='*' if chosen else 'o',linestyle='none',
                        color='#9e32af',markersize=11 if chosen else 4,alpha=1 if chosen else .5)
                if rank < 3:
                    ax.text(p[0]+.15,p[1]-.3,candidate['id'],fontsize=7,color='#862b96')
                if chosen:
                    points=[candidate['path']['start_enu_m'],*candidate['path']['waypoints']]
                    ax.plot([p[0] for p in points],[p[1] for p in points],'--',color='#9e32af',linewidth=1)
    axes[0].set_title('几何观察覆盖：蓝=已观测中心；灰=未观测；深灰=不计入统计')
    axes[1].set_title('显式任务关系产生的语义场；箭头=未定位方位线索')
    fig.suptitle('{} | 有效照片 {} | 几何覆盖 {:.1%} | 地标 {} / 未定位线索 {}'.format(
        snapshot.get('evidence_mode','archived actual observations'),snapshot['observation_count'],
        snapshot['geometric_coverage_fraction'],snapshot['localized_landmarks'],snapshot['unlocalized_bearing_clues']),fontsize=11)
    footnote = '覆盖不等于识别成功；椭圆为未标定 3σ 几何包络；绿色为记录的规划路径。'
    if prediction:
        footnote += ' 紫色点/星/虚线为 NBV 候选/选点/路径预测，未执行。'
    fig.text(.5,.015,footnote,ha='center',fontsize=8)
    fig.tight_layout(rect=[0,.045,1,.93])
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(str(output),dpi=160);fig.savefig(str(output.with_suffix('.svg')));plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--world',type=Path)
    args=parser.parse_args();render(json.loads(args.snapshot.read_text()),args.output,args.world)
