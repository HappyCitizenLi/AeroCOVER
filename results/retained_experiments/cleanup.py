#!/usr/bin/env python3
"""Explicit retained-data inventory and guarded deletion plan."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent
WS = ROOT.parents[1]
SU = Path('/media/uav/SU710')
VERSIONS = {
    'AeroCOVER-V8': 'aerocover_four_scene_comparison_v8',
    'V13': 'vofod_open_floor_second_v13',
    'V14': 'vofod_open_floor_first_v14',
    'V15': 'vofod_floor_first_all_v15',
    'V16': 'vofod_office_os1_ready001_v16',
}
OBSOLETE_LOCAL = [
    'aerocover_four_scene_comparison', 'aerocover_four_scene_comparison_v2',
    'aerocover_four_scene_comparison_v3', 'aerocover_four_scene_comparison_v4',
    'aerocover_four_scene_comparison_v5', 'aerocover_four_scene_comparison_v6',
    'aerocover_four_scene_comparison_v7', 'aerocover_open_19m',
    'aerocover_open_19m_strict', 'aerocover_paper_main_comparison',
    'aerocover_paper_stinit_native_mapexec', 'vofod_after_v10_summary',
    'vofod_map_floor_third_layer_v10', 'vofod_ready_three_v11',
    'vofod_ready_three_v12', 'vofod_state_time_pair_v9',
]
OBSOLETE_SU = [
    'aerocover_cleanup_20260910', 'aerocover_cleanup_20260913',
    'aerocover_followup_20260910', 'aerocover_followup_20260911',
    'aerocover_four_scene_20260912', 'aerocover_four_scene_comparison_v4_20260913',
    'aerocover_four_scene_v2_20260912', 'aerocover_four_scene_v3_20260913',
    'aerocover_gate1p5_mt3_v7', 'aerocover_open_19m_20260913',
    'aerocover_original_mid360_retired_20260912',
    'aerocover_runtime_round3_20260911', 'aerocover_runtime_round4_20260912',
    'aerocover_time_semantics_v6_20260913', 'vofod_map_floor_third_layer_v10',
    'vofod_ready_three_v11', 'vofod_ready_three_v12', 'vofod_state_time_pair_v9',
]
GARBAGE_PACKS = ['tmp_pack_KIl0Lw', 'tmp_pack_PGZTE3', 'tmp_pack_T2q2Ep',
                 'tmp_pack_cbaogj', 'tmp_pack_pMEEeg', 'tmp_pack_xMxXVj']


def read(p):
    return json.loads(p.read_text())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def files_under(p):
    if p.is_symlink() or p.is_file():
        return [p]
    out=[]
    for parent, dirs, files in os.walk(p, followlinks=False):
        parent=Path(parent)
        out += [parent/x for x in files]
        out += [parent/x for x in dirs if (parent/x).is_symlink()]
    return out


def direct_sources(root, sources):
    target=root/'sources'
    if target.is_symlink():
        target.unlink()
    target.mkdir(exist_ok=True)
    for scene, source in sources.items():
        link=target/scene
        if link.is_symlink():
            link.unlink()
        assert not link.exists(), link
        link.symlink_to(source, target_is_directory=True)


def prepare():
    assert not (ROOT/'retained_index.json').exists(), 'Already prepared; use verify/plan'
    subprocess.run(['git','fsck','--connectivity-only','--no-dangling'],cwd=WS,check=True)
    entries=[]; sources={}; seen={}
    (ROOT/'configs').mkdir(exist_ok=True)
    (ROOT/'sources').mkdir(exist_ok=True)
    for version,name in VERSIONS.items():
        root=WS/'results'/name
        for mp in sorted((root/'runs').glob('*/*/run_manifest.json')):
            d=mp.parent; scene,method=d.parent.name,d.name
            if version=='AeroCOVER-V8' and not method.startswith('AeroCOVER-'):
                continue
            m=read(mp);metrics=d/'metrics.json'
            assert m['status']=='COMPLETE' and sha(metrics)==m['metrics_sha256'], mp
            source=Path(m['source']).parent.resolve()
            assert (source/'source.bag').is_file() and Path(m['scenario']).is_file()
            assert sources.setdefault(scene,str(source)) == str(source)
            candidates=[Path(m['config'])]
            for sub in ('config_snapshot','configs','code_snapshot'):
                if (root/sub).exists():
                    candidates += list((root/sub).rglob('*.yaml'))
            config=next(p for p in candidates if p.is_file() and sha(p)==m['config_sha256'])
            frozen=ROOT/'configs'/f'{version}_{scene}_{method}.yaml'
            shutil.copy2(config,frozen)
            identity=str(mp.resolve()); owner=seen.setdefault(identity,f'{version}/{scene}/{method}')
            inventory=[]
            for p in files_under(d.resolve()):
                if p.is_symlink():
                    raise AssertionError(f'Unexpected symlink inside run: {p}')
                st=p.stat()
                # Hash small evidence fully; preserve byte count/mtime for large bags.
                inventory.append(dict(path=str(p),size=st.st_size,mtime_ns=st.st_mtime_ns,
                                      sha256=sha(p) if st.st_size<8*1024*1024 else None))
            entries.append(dict(version=version,root=str(root),scene=scene,method=method,
                                run_dir=str(d),owner=owner,config=str(frozen),
                                config_sha256=m['config_sha256'],manifest_sha256=sha(mp),
                                metrics_sha256=sha(metrics),output_bag_existed=(d/'output.bag').is_file(),
                                inventory=inventory))
    assert len(entries)==19 and len(seen)==18 and len(sources)==4
    assert sum(e['version']=='AeroCOVER-V8' for e in entries)==8
    for root in [ROOT]+[WS/'results'/name for name in VERSIONS.values()]:
        direct_sources(root,sources)
    source_inventory=[]
    for source in sources.values():
        for p in files_under(Path(source)):
            st=p.stat();source_inventory.append(dict(path=str(p),size=st.st_size,mtime_ns=st.st_mtime_ns,
                                                       sha256=sha(p) if st.st_size<8*1024*1024 else None))
    # Keep original retained-version documents as provenance, not live dependencies.
    for name in VERSIONS.values():
        root=WS/'results'/name
        archive=root/'code_snapshot'/'pre_cleanup_reports'
        archive.mkdir(parents=True,exist_ok=True)
        for fn in ('README_ZH.md','PER_SEQUENCE_ZH.md','replay.py'):
            p=root/fn
            if p.exists():shutil.copy2(p,archive/fn)
    # V8 audit is retained only for AeroCOVER; other methods are retired.
    p=WS/'results'/VERSIONS['AeroCOVER-V8']/'ERROR_AUDIT.json'
    a=read(p);a['summary']=[r for r in a['summary'] if r['method'].startswith('AeroCOVER-')]
    p.write_text(json.dumps(a,ensure_ascii=False,indent=2)+'\n')
    (ROOT/'retained_index.json').write_text(json.dumps(dict(entries=entries,sources=sources,source_inventory=source_inventory),ensure_ascii=False,indent=2)+'\n')
    print('Prepared 19 logical records / 18 unique runs, 4 direct source links')


def verify():
    index=read(ROOT/'retained_index.json')
    for e in index['entries']:
        d=Path(e['run_dir'])
        assert sha(d/'run_manifest.json')==e['manifest_sha256'],d
        assert sha(d/'metrics.json')==e['metrics_sha256'],d
        assert sha(Path(e['config']))==e['config_sha256']
        assert (d/'output.bag').is_file()==e['output_bag_existed'],d
        assert (Path(e['root'])/'sources'/e['scene']).resolve()==Path(index['sources'][e['scene']])
    inventories=[x for e in index['entries'] for x in e['inventory']]+index['source_inventory']
    for item in inventories:
        p=Path(item['path']);st=p.stat()
        assert st.st_size==item['size'] and st.st_mtime_ns==item['mtime_ns'],p
        if item['sha256']:assert sha(p)==item['sha256'],p
    print('PASS: all retained run files and 4 sources unchanged; source links direct')
    return index


def plan():
    index=verify()
    targets=[WS/'results'/x for x in OBSOLETE_LOCAL]+[SU/x for x in OBSOLETE_SU]
    targets += [SU/'lyk_results/detrack_vofod_c15_2m_1seed',
                SU/'aerocover_four_scene_v4_flow_20260913/probes/OPEN',
                SU/'aerocover_four_scene_v4_flow_20260913/scenarios',
                SU/'aerocover_open_19m_strict_20260913/sources/OPEN',
                SU/'aerocover_open_19m_strict_20260913/sources/OPEN_entry',
                SU/'aerocover_open_19m_strict_20260913/runs',
                SU/'aerocover_open_19m_strict_20260913/scenarios',
                SU/'aerocover_open_19m_strict_20260913/PER_SEQUENCE_ZH.md',
                SU/'aerocover_open_19m_strict_20260913/README_ZH.md']
    for scene in index['sources']:
        for d in (SU/'aerocover_twenty_v8/runs'/scene).iterdir():
            if d.name.startswith('VoFOD-'):targets.append(d)
    targets += [WS/'.git/objects/pack'/x for x in GARBAGE_PACKS]
    v8=WS/'results'/VERSIONS['AeroCOVER-V8']
    targets += [v8/'ERROR_CAUSES_ZH.md',v8/'STATE_TIME_REEVALUATION_PLAN_ZH.md']
    # These are inactive, unrun historical parameter alternatives, not protected code.
    targets += list((WS/'results'/VERSIONS['V15']/'config_snapshot').glob('*_v2.yaml'))
    protected=[ROOT,WS/'src',WS/'.git/HEAD',WS/'.git/refs']
    protected += [Path(x) for x in index['sources'].values()]
    protected += [Path(e['root']) for e in index['entries']]
    protected += [Path(e['run_dir']).resolve() for e in index['entries']]
    protected_full=[ROOT,WS/'src',WS/'.git/HEAD',WS/'.git/refs']
    protected_full += [Path(x) for x in index['sources'].values()]
    protected_full += [Path(e['run_dir']).resolve() for e in index['entries']]
    candidates=[]
    for p in targets:
        assert p.is_absolute() and p not in (WS,WS/'results',SU,WS/'.git',WS/'.git/objects',WS/'.git/objects/pack')
        if not p.exists() and not p.is_symlink():continue
        assert not p.is_symlink(), f'Top-level deletion target must be a resolved regular path: {p}'
        assert all(p!=q and p not in q.parents for q in protected),f'Protected dependency: {p}'
        assert all(q not in p.parents for q in protected_full),f'Inside protected data: {p}'
        assert p.resolve()==p, f'Unexpected redirect: {p}'
        listing=files_under(p)
        size=sum(f.lstat().st_blocks*512 for f in listing)
        st=p.stat()
        candidates.append(dict(path=str(p),bytes_allocated=size,file_count=len(listing),
                               device=st.st_dev,inode=st.st_ino,mtime_ns=st.st_mtime_ns))
    assert len({r['path'] for r in candidates})==len(candidates)
    for a in candidates:
        assert not any(Path(a['path']) in Path(b['path']).parents for b in candidates if a!=b)
    p=dict(targets=candidates,protected=[str(p) for p in protected],protected_full=[str(p) for p in protected_full],
           bytes_allocated=sum(x['bytes_allocated'] for x in candidates),
           branches='Only master exists; no branch deletion or history rewrite',
           reversible=False)
    (ROOT/'deletion_plan.json').write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n')
    print(f"PLAN {len(candidates)} targets, {p['bytes_allocated']/2**30:.2f} GiB allocated")
    for x in candidates:print(f"{x['bytes_allocated']/2**30:8.3f} GiB  {x['path']}")


def execute():
    verify()
    p=read(ROOT/'deletion_plan.json')
    assert not (ROOT/'deletion_completed.json').exists()
    subprocess.run(['git','fsck','--connectivity-only','--no-dangling'],cwd=WS,check=True)
    git_processes=subprocess.run(['ps','-C','git','-o','args='],capture_output=True,text=True).stdout
    assert not any(word in git_processes for word in ('pack-objects','repack','index-pack',' gc')), git_processes
    before={str(x):shutil.disk_usage(x)._asdict() for x in (WS,SU)}
    for item in p['targets']:
        target=Path(item['path']);st=target.stat()
        assert not target.is_symlink() and target.resolve()==target
        assert (st.st_dev,st.st_ino,st.st_mtime_ns)==(item['device'],item['inode'],item['mtime_ns']),target
        assert all(target!=Path(q) and target not in Path(q).parents for q in p['protected'])
        assert all(Path(q) not in target.parents for q in p['protected_full'])
    completed=[]
    for item in p['targets']:
        target=Path(item['path'])
        if target.is_dir():shutil.rmtree(target)
        else:target.unlink()
        completed.append(item)
        (ROOT/'deletion_progress.json').write_text(json.dumps(completed,ensure_ascii=False,indent=2)+'\n')
        print('DELETED',target,flush=True)
    verify()
    subprocess.run(['git','fsck','--connectivity-only','--no-dangling'],cwd=WS,check=True)
    after={str(x):shutil.disk_usage(x)._asdict() for x in (WS,SU)}
    (ROOT/'deletion_completed.json').write_text(json.dumps(dict(deleted=completed,before=before,after=after),ensure_ascii=False,indent=2)+'\n')
    print('COMPLETE; retained data and Git connectivity verified')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['prepare','verify','plan','execute'])
    globals()[parser.parse_args().stage]()
