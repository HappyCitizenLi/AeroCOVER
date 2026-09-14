#!/usr/bin/env python3
"""Audit existing CSVs only; no algorithm runs or metric changes."""
import ast
import csv
import json
from collections import defaultdict, Counter
from pathlib import Path
import sys

import numpy as np
import rosbag

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[1]/'src/soft_vofod_evaluation/scripts'))
from evaluate_bag import assignments


def main():
    mt_truth=defaultdict(list)
    with rosbag.Bag(str(ROOT/'sources/MT/source.bag')) as bag:
        topics={f'/mid360_multi_uav_sim/ground_truth/uav{i}/odom':f'uav{i}' for i in (2,3)}
        for topic,m,_ in bag.read_messages(topics=list(topics)):
            p=m.pose.pose.position
            mt_truth[topics[topic]].append([m.header.stamp.to_sec(),p.x,p.y,p.z])
    mt_truth={u:np.array(v) for u,v in mt_truth.items()}
    summary=[];fp_details=[]
    for path in sorted((ROOT/'runs').glob('*/*/metrics.json')):
        directory=path.parent;m=json.loads(path.read_text())
        scene=directory.parent.name;method=directory.name
        timeline=list(csv.DictReader((directory/'tracking_timeline.csv').open()))
        by_frame=defaultdict(list);by_truth=defaultdict(list);ever_matched=set()
        for r in timeline:
            by_frame[round(float(r['stamp']),9)].append(r);by_truth[r['truth_id']].append(r)
            if r['matched_track_id']:ever_matched.add(r['matched_track_id'])
        tracks=defaultdict(list)
        for r in csv.DictReader((directory/'track_timeseries.csv').open()):
            if r['state']!='active':continue
            r['position']=ast.literal_eval(r['position'])
            tracks[round(float(r['stamp']),9)].append(r)
        hidden=defaultdict(list)
        for r in csv.DictReader((directory/'los_ignored_timeseries.csv').open()):hidden[round(float(r['stamp']),9)].append(r['truth_id'])
        fp=[]
        for stamp,predictions in tracks.items():
            matched={r['matched_track_id'] for r in by_frame[stamp] if r['matched_track_id']}
            unmatched=[p for p in predictions if p['id'] not in matched]
            ignored_truth=[]
            for u in hidden[stamp]:
                a=mt_truth[u];assert a[0,0]<=stamp<=a[-1,0]
                ignored_truth.append(dict(id=u,position=tuple(np.interp(stamp,a[:,0],a[:,k]) for k in (1,2,3))))
            ignored={col for _,col,_ in assignments(ignored_truth,unmatched,1.5)}
            for i,p in enumerate(unmatched):
                if i in ignored:continue
                fp.append(p)
                if scene=='MT' and method=='AeroCOVER-Mid360':
                    distances={u:float(np.linalg.norm(np.array(p['position'])-[np.interp(stamp,a[:,0],a[:,k]) for k in (1,2,3)])) for u,a in mt_truth.items()}
                    fp_details.append(dict(stamp=stamp,id=p['id'],position=p['position'],stale_s=float(p['stale_s']),distances=distances))
        assert len(fp)==m['track_set']['fp'],(scene,method,len(fp),m['track_set']['fp'])
        fn=Counter();initial=0;late=0;never=0
        for u,rows in by_truth.items():
            hits=[i for i,r in enumerate(rows) if r['matched_track_id']]
            for i,r in enumerate(rows):
                if r['matched_track_id']:continue
                fn[u]+=1
                if not hits:never+=1
                elif i<hits[0]:initial+=1
                else:late+=1
        diagnostics=list(csv.DictReader((directory/'diagnostics_timeseries.csv').open()))
        detections=[float(r.get('detection_count',0)) for r in diagnostics]
        summary.append(dict(scene=scene,method=method,fn_by_target=dict(fn),
                            fn_initial=initial,fn_after_first_match=late,fn_never_matched=never,
                            fp=len(fp),fp_never_matched_id=sum(p['id'] not in ever_matched for p in fp),
                            fp_stale_over_1s=sum(float(p['stale_s'])>1 for p in fp),
                            fp_within_0p5m_ground=sum(abs(p['position'][2])<.5 for p in fp),
                            fp_z_median=float(np.median([p['position'][2] for p in fp])) if fp else None,
                            fn_before_ready=sum(not r['matched_track_id'] and
                                                float(r['stamp']) < m.get('initialization',{}).get('detection_ready_stamp_s',0)
                                                for r in timeline),
                            detection_frames=sum(v>0 for v in detections),detection_total=sum(detections)))
    (ROOT/'ERROR_AUDIT.json').write_text(json.dumps(dict(summary=summary,mt_aerocover_fp=fp_details),ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(summary=summary,mt_aerocover_fp=fp_details),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
