#!/usr/bin/env python3
"""Generate printable unlabeled actual-capture contact sheets; no model or labels."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.backends.backend_pdf import PdfPages


def render(inventory, root, output):
    root=Path(root).resolve();data=json.loads(Path(inventory).read_text());rows=data['images']
    if data.get('status')!='unlabelled_inventory_not_evaluation_set':
        raise ValueError('requires unlabelled inventory')
    if Path(output).exists():raise ValueError('refusing to overwrite review file')
    with PdfPages(output) as pdf:
        for offset in range(0,len(rows),4):
            fig,axes=plt.subplots(2,2,figsize=(11.7,8.3));fig.suptitle('Unlabelled Gazebo capture inventory - human review required',fontsize=12)
            for axis,row in zip(axes.flat,rows[offset:offset+4]):
                path=(root/row['image_path']).resolve()
                try:path.relative_to(root)
                except ValueError:raise ValueError('image outside root')
                axis.imshow(mpimg.imread(str(path)));axis.set_axis_off()
                axis.set_title('{}  scene={}  seed={}'.format(row['capture_id'],row['scene_id'],row['seed_evidence']['seed']),fontsize=7)
            for axis in list(axes.flat)[len(rows[offset:offset+4]):]:axis.set_axis_off()
            fig.tight_layout(rect=(0,0,1,.95));pdf.savefig(fig);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inventory',required=True);p.add_argument('--root',required=True);p.add_argument('--output',required=True);a=p.parse_args();render(a.inventory,a.root,a.output)
