"""Cross compile immutable, host-generated SDK C type support against Q1 ROS headers.

Run inside the development container. Never executes a target binary.
"""
from pathlib import Path
import concurrent.futures
import json
import re
import shlex
import shutil
import subprocess

ROOT=Path('/opt/q1')
NATIVE=Path('/opt/q1_ros/build/aimdk_msgs')
ARM_ROS=ROOT/'cross/rootfs/opt/ros/humble'
OUT=ROOT/'cross/aimdk_msgs'


def run(argv):
    subprocess.run(list(map(str,argv)),check=True)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'lib').mkdir(exist_ok=True)
    for folder in ['include','share']:
        shutil.copytree(Path('/opt/q1_ros/install/aimdk_msgs')/folder,OUT/folder,dirs_exist_ok=True)
    report=[]
    for suffix in ['rosidl_generator_c','rosidl_typesupport_c','rosidl_typesupport_introspection_c','rosidl_typesupport_fastrtps_c']:
        name='aimdk_msgs__'+suffix
        directory=NATIVE/'CMakeFiles'/(name+'.dir')
        flags=(directory/'flags.make').read_text()
        lang='CXX' if 'CXX_FLAGS =' in flags else 'C'
        compiler='aarch64-linux-gnu-g++' if lang=='CXX' else 'aarch64-linux-gnu-gcc'
        args=[]
        for field in ['DEFINES','INCLUDES','FLAGS']:
            line=re.search(r'^'+lang+'_'+field+r' = (.*)$',flags,re.M)
            if line:
                args.extend(shlex.split(line.group(1).replace('/opt/ros/humble',str(ARM_ROS))))
        link=shlex.split((directory/'link.txt').read_text())
        objects=[v for v in link if v.endswith('.o')]
        jobs=[]
        for obj in objects:
            source=NATIVE/obj.split('.dir/',1)[1][:-2]
            target=OUT/'objects'/obj
            target.parent.mkdir(parents=True,exist_ok=True)
            if not target.exists() or target.stat().st_mtime<source.stat().st_mtime:
                jobs.append([compiler,*args,'-c',source,'-o',target])
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            list(executor.map(run,jobs))
        tail=link[link.index(objects[-1])+1:]
        libraries=[]
        for value in tail:
            if value.startswith('-Wl,-rpath,') or value.startswith('-Wl,-rpath-link,'):
                continue
            value=value.replace('/opt/ros/humble',str(ARM_ROS))
            if value.startswith('libaimdk_msgs'):
                value=str(OUT/'lib'/value)
            libraries.append(value)
        output=OUT/'lib'/('lib'+name+'.so')
        run([compiler,'-shared','-fPIC','-Wl,-soname,'+output.name,'-o',output,
             *[OUT/'objects'/x for x in objects],*libraries,
             '-Wl,-rpath-link,'+str(ARM_ROS/'lib')])
        machine=subprocess.check_output(['aarch64-linux-gnu-readelf','-h',output],text=True)
        assert 'AArch64' in machine
        report.append({'library':output.name,'sources':len(objects),'machine':'AArch64'})
    (OUT/'cross-build.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    main()
