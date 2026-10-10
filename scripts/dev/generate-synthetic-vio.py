#!/usr/bin/env python3
"""Independent analytic motion, ray-cast images and IMU. No estimator imports."""
import argparse
import hashlib
import json
import math
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation

VERSION = 'synthetic-vio-2'
ROOT = pathlib.Path(__file__).resolve().parents[2]
SCENARIOS = ('clean', 'mobile40', 'mobile60', 'stationary', 'pure-rotation', 'constant-velocity')

def sha(data):
    return hashlib.sha256(data).hexdigest()

def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()

def write_new(path, data):
    with path.open('xb') as output:
        output.write(data)

def surface(center, u, v, half_extent, texture):
    return {'center': list(center), 'u': list(u), 'v': list(v), 'half_extent': list(half_extent), 'texture': texture}

def make_spec(seed=101, scenario='clean', portrait=False):
    if scenario not in SCENARIOS or seed < 0:
        raise ValueError('Unsupported scenario or seed')
    camera = {'width': 480 if portrait else 320, 'height': 640 if portrait else 240,
              'fx': 465.60288917518244 if portrait else 260., 'fy': 465.60288917518244 if portrait else 255.,
              'cx': 239.5 if portrait else 159.5, 'cy': 319.5 if portrait else 119.5,
              'modelType': 2, 'k2': 0., 'k3': 0., 'k4': 0., 'k5': 0.,
              'r_ic': [1,0,0,0,0,1,0,-1,0], 't_ic': [.02,-.01,-.02]}
    if scenario in ('pure-rotation','constant-velocity'):
        camera['t_ic'] = [0,0,0]
    rng = np.random.default_rng(np.random.SeedSequence([seed, 20]))
    phases = [0.,0.,0.] if seed == 101 else rng.uniform(-.35,.35,3).tolist()
    depth_shift = float(np.random.default_rng(np.random.SeedSequence([seed,10])).uniform(-.15,.15))
    surfaces = [surface([0,6+depth_shift,1.5],[1,0,0],[0,0,1],[4,3],0),
                surface([0,4,0],[1,0,0],[0,1,0],[4,4],1),
                surface([-3.5,4,1.6],[0,1,0],[0,0,1],[4,1.6],2),
                surface([3.5,4,1.6],[0,1,0],[0,0,1],[4,1.6],3)]
    # Two boxes at distinct depths supply real parallax and foreground occlusion.
    for x,y,z,hx,hy,hz in [(-1.,3.4+depth_shift,1.0,.38,.4,.65),(.9,4.4-depth_shift,1.5,.5,.35,.55)]:
        faces = [( [x,y-hy,z],[1,0,0],[0,0,1],[hx,hz]),
                 ( [x-hx,y,z],[0,1,0],[0,0,1],[hy,hz]),
                 ( [x+hx,y,z],[0,1,0],[0,0,1],[hy,hz]),
                 ( [x,y,z+hz],[1,0,0],[0,1,0],[hx,hy])]
        for center,u,v,extent in faces:
            surfaces.append(surface(center,u,v,extent,len(surfaces)))
    config = dict(camera, acc_n=.5, acc_w=.003, gyr_n=.2, gyr_w=.0002, g_norm=9.81)
    mobile = scenario.startswith('mobile')
    noise = {'acc_density': .005 if mobile else 0., 'gyro_density': .0002 if mobile else 0.,
             'acc_random_walk_density': .0002 if mobile else 0., 'gyro_random_walk_density': .00002 if mobile else 0.,
             'acc_bias': [.03,-.02,.01] if mobile else [0.,0.,0.],
             'gyro_bias': [.001,-.0015,.0005] if mobile else [0.,0.,0.],
             'acc_quantization': .1 if mobile else 0., 'gyro_quantization_rad_s': math.radians(.1) if mobile else 0.,
             'measurement_density_units': 'SI per sqrt(Hz); sample std=density/sqrt(dt)',
             'random_walk_units': 'SI per sqrt(s); increment std=density*sqrt(dt)'}
    return {'schema':'mobile-slam-synthetic-spec-v1','generator_version':VERSION,
            'generator_sha256':sha(pathlib.Path(__file__).read_bytes()),'seed':seed,'scenario':scenario,
            'fixture_kind':'frozen_acceptance','duration_s':20.,'image_hz':20,'frame_count':400,
            'imu_hz':int(scenario[6:]) if mobile else 200, 'gravity_physical':[0,0,-9.81],
            'camera':camera, 'motion':{'position_origin':[0,0,1.2],'translation_amplitude':[.4,.3,.2],
                'translation_frequency':[1.3,1.1,1.7],'translation_phase':phases,
                'rotation_amplitude':[.08,.12,.18],'rotation_frequency':[.7,.9,.55],
                'rotation_phase':[phases[0],.2+phases[1],phases[2]],
                'constant_velocity':[.15,.06,.02], 'start_s':1.,'stop_s':19.,'ramp_s':1.},
            'scene':{'surfaces':surfaces,'texture_seed_stream':30,'texture_size':256,'texture_cells':64,
                     'texture_filter':'bilinear','background':16,'landmark_grid':[12,9]},
            'injected_noise':noise,
            'estimator_profile':{'name':'frozen_legacy_mobile_covariance_fixed_work',
                'config':config,'parameters':{'mobile':[10.,10,120],'tracking':[21,3,20,0.],
                    'pnp':[False,3],'fThreshold':1.,'execution':[0,1]},
                'derived':{'keyframe_parallax_pixels':10.,'initial_depth_m':2.,'equalize':1,'fisheye':0,
                    'lk_iterations':20,'lk_epsilon':.03,'window_size':10,'feature_capacity':1000},
                'noise_parameter_semantics':'existing legacy sample-sigma parameters, distinct from injected density'},
            'acceptance':{'warmup_start_s':5.,'source_end_s':20.,'scheduled_warmup_frames':300,
                'association_tolerance_s':.0005,'rpe_interval_s':1.,'rpe_tolerance_s':.05,
                'ate_rmse_m':.25 if mobile else .10,'ape_rmse_deg':5. if mobile else 3.,
                'postwarmup_coverage':.80 if mobile else .90,'first_pose_latest_s':5.,
                'clean_rpe_m':.10,'clean_rpe_deg':3.,'expected_degenerate':scenario in ('stationary','pure-rotation','constant-velocity')},
            'coordinate_contract':{'poses':'T_W_B and T_W_C','rotation':'row-major 3x3','quaternion':'xyzw',
                'imu':'body specific force m/s^2, angular velocity rad/s','timestamps':'seconds; CSV integer ns',
                'camera_extrinsic':'T_B_C','pixel_center':'integer centers; native OpenCV convention'}}

def time_warp(t, motion):
    start,stop,ramp = motion['start_s'],motion['stop_s'],motion['ramp_s']
    if t <= start:return 0.,0.,0.
    s=(t-start)/ramp
    def smooth(x):return 10*x**3-15*x**4+6*x**5
    def derivative(x):return 30*x*x-60*x**3+30*x**4
    def integral(x):return 2.5*x**4-3*x**5+x**6
    if t < start+ramp:return ramp*integral(s),smooth(s),derivative(s)/ramp
    flat_end=stop-ramp
    if t <= flat_end:return t-start-.5*ramp,1.,0.
    if t < stop:
        s=(t-flat_end)/ramp
        return flat_end-start-.5*ramp+ramp*(s-integral(s)),1-smooth(s),-derivative(s)/ramp
    return stop-start-ramp,0.,0.

def body_state(t, spec):
    motion=spec['motion'];u,du,ddu=time_warp(float(t),motion)
    amp=np.array(motion['translation_amplitude']);freq=np.array(motion['translation_frequency']);phase=np.array(motion['translation_phase'])
    angles=freq*u+phase
    shape=np.array([np.sin(angles[0])-np.sin(phase[0]),np.cos(phase[1])-np.cos(angles[1]),np.sin(angles[2])-np.sin(phase[2])])
    first=np.array([freq[0]*np.cos(angles[0]),freq[1]*np.sin(angles[1]),freq[2]*np.cos(angles[2])])
    second=np.array([-freq[0]**2*np.sin(angles[0]),freq[1]**2*np.cos(angles[1]),-freq[2]**2*np.sin(angles[2])])
    p=np.array(motion['position_origin'])+amp*shape;v=amp*first*du;a=amp*(second*du*du+first*ddu)
    ra=np.array(motion['rotation_amplitude']);rf=np.array(motion['rotation_frequency']);rp=np.array(motion['rotation_phase'])
    phi,theta,psi=ra*(np.sin(rf*u+rp)-np.sin(rp));pd,td,yd=ra*rf*np.cos(rf*u+rp)*du
    if spec['scenario']=='stationary':
        p=np.array(motion['position_origin']);v=np.zeros(3);a=np.zeros(3);phi=theta=psi=pd=td=yd=0.
    elif spec['scenario']=='pure-rotation':p=np.array(motion['position_origin']);v=np.zeros(3);a=np.zeros(3)
    elif spec['scenario']=='constant-velocity':
        v=np.array(motion['constant_velocity']);p=np.array(motion['position_origin'])+v*t;a=np.zeros(3)
    cp,sp,ct,st,cy,sy=np.cos(phi),np.sin(phi),np.cos(theta),np.sin(theta),np.cos(psi),np.sin(psi)
    rx=np.array([[1,0,0],[0,cp,-sp],[0,sp,cp]])
    ry=np.array([[ct,0,st],[0,1,0],[-st,0,ct]])
    rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    r=rz@ry@rx
    gyro=np.array([pd-yd*st,td*cp+yd*sp*ct,-td*sp+yd*cp*ct])
    return {'position':p,'velocity':v,'acceleration':a,'rotation':r,'gyro':gyro,
            'accel_measurement':r.T@(a-np.array(spec['gravity_physical']))}

def camera_pose(state,spec):
    cam=spec['camera'];rbc=np.array(cam['r_ic']).reshape(3,3)
    return state['rotation']@rbc,state['position']+state['rotation']@np.array(cam['t_ic'])

def project_points(points,state,spec):
    rwc,pwc=camera_pose(state,spec);cam=spec['camera'];rbc=np.array(cam['r_ic']).reshape(3,3)
    xyz=(points-pwc)@rwc
    camera_velocity=state['velocity']+state['rotation']@np.cross(state['gyro'],np.array(cam['t_ic']))
    wc=rbc.T@state['gyro']
    derivatives=-np.cross(wc,xyz)-(rwc.T@camera_velocity)
    with np.errstate(divide='ignore',invalid='ignore'):
        normalized=xyz[:,:2]/xyz[:,2,None]
        velocity=(derivatives[:,:2]*xyz[:,2,None]-xyz[:,:2]*derivatives[:,2,None])/xyz[:,2,None]**2
    pixels=normalized*np.array([cam['fx'],cam['fy']])+np.array([cam['cx'],cam['cy']])
    return {'xyz':xyz,'normalized':normalized,'pixels':pixels,'velocity':velocity}

def textures(spec):
    result=[];scene=spec['scene'];factor=scene['texture_size']//scene['texture_cells']
    for i in range(len(scene['surfaces'])):
        rng=np.random.default_rng(np.random.SeedSequence([spec['seed'],scene['texture_seed_stream'],i]))
        values=rng.integers(25,231,(scene['texture_cells'],scene['texture_cells']),dtype=np.uint8)
        result.append(np.repeat(np.repeat(values,factor,axis=0),factor,axis=1))
    return result

def cast_rays(origin,rays,surfaces,texture_images):
    depth=np.full(len(rays),np.inf);gray=np.full(len(rays),16.,dtype=float);ids=np.full(len(rays),-1,dtype=int)
    for index,face in enumerate(surfaces):
        center=np.array(face['center']);u=np.array(face['u']);v=np.array(face['v']);normal=np.cross(u,v)
        denominator=rays@normal
        distance=np.divide((center-origin)@normal,denominator,out=np.full(len(rays),np.inf),where=abs(denominator)>1e-12)
        possible=(distance>1e-8)&(distance<depth)
        indices=np.flatnonzero(possible)
        if not len(indices):continue
        offsets=origin+distance[indices,None]*rays[indices]-center
        uv=np.column_stack((offsets@u,offsets@v));extent=np.array(face['half_extent'])
        inside=np.all(abs(uv)<=extent+1e-12,axis=1);indices=indices[inside];uv=uv[inside]
        if not len(indices):continue
        texture=texture_images[face['texture']];h,w=texture.shape
        xy=(uv/extent+1)*.5*np.array([w-1,h-1]);xy=np.clip(xy,0,[w-1,h-1])
        lo=np.floor(xy).astype(int);hi=np.minimum(lo+1,[w-1,h-1]);alpha=xy-lo
        sampled=(1-alpha[:,0])*(1-alpha[:,1])*texture[lo[:,1],lo[:,0]]+alpha[:,0]*(1-alpha[:,1])*texture[lo[:,1],hi[:,0]]+(1-alpha[:,0])*alpha[:,1]*texture[hi[:,1],lo[:,0]]+alpha[:,0]*alpha[:,1]*texture[hi[:,1],hi[:,0]]
        gray[indices]=sampled;depth[indices]=distance[indices];ids[indices]=index
    return np.rint(gray).astype(np.uint8),depth,ids

def render(state,spec,texture_images):
    cam=spec['camera'];yy,xx=np.mgrid[:cam['height'],:cam['width']]
    rays=np.column_stack(((xx.ravel()-cam['cx'])/cam['fx'],(yy.ravel()-cam['cy'])/cam['fy'],np.ones(xx.size)))
    r,p=camera_pose(state,spec)
    gray,depth,ids=cast_rays(p,rays@r.T,spec['scene']['surfaces'],texture_images)
    return gray.reshape(cam['height'],cam['width']),depth.reshape(cam['height'],cam['width']),ids.reshape(cam['height'],cam['width'])

def landmarks(spec):
    points=[];gx,gy=spec['scene']['landmark_grid']
    for face in spec['scene']['surfaces']:
        center,u,v=np.array(face['center']),np.array(face['u']),np.array(face['v']);extent=face['half_extent']
        for a in np.linspace(-.9,.9,gx):
            for b in np.linspace(-.9,.9,gy):points.append(center+a*extent[0]*u+b*extent[1]*v)
    return np.array(points)

def visible_features(points,state,spec,texture_images):
    projection=project_points(points,state,spec);cam=spec['camera'];pixels=projection['pixels'];xyz=projection['xyz']
    keep=(xyz[:,2]>0)&(pixels[:,0]>=4)&(pixels[:,0]<cam['width']-4)&(pixels[:,1]>=4)&(pixels[:,1]<cam['height']-4)
    indices=np.flatnonzero(keep)
    if not len(indices):return []
    r,p=camera_pose(state,spec)
    rays=np.column_stack((projection['normalized'][indices],np.ones(len(indices))))@r.T
    _,depth,_=cast_rays(p,rays,spec['scene']['surfaces'],texture_images)
    indices=indices[abs(depth-xyz[indices,2])<=1e-7]
    return [(int(i),[*projection['normalized'][i],1.,*pixels[i],*projection['velocity'][i]]) for i in indices]

def sample_imu(spec):
    hz=spec['imu_hz'];dt=1/hz;noise=spec['injected_noise'];rng=np.random.default_rng(np.random.SeedSequence([spec['seed'],40]))
    bias=np.array(noise['acc_bias']+noise['gyro_bias'],dtype=float)
    density=np.array([noise['acc_density']]*3+[noise['gyro_density']]*3)
    walk=np.array([noise['acc_random_walk_density']]*3+[noise['gyro_random_walk_density']]*3)
    quant=np.array([noise['acc_quantization']]*3+[noise['gyro_quantization_rad_s']]*3)
    samples=[];truth=[]
    for i in range(math.ceil(spec['duration_s']*hz)+1):
        t=i/hz;s=body_state(t,spec);value=np.r_[s['accel_measurement'],s['gyro']]
        if i:bias+=walk*math.sqrt(dt)*rng.standard_normal(6)
        observed=value+bias+density/math.sqrt(dt)*rng.standard_normal(6)
        enabled=quant>0;observed[enabled]=np.rint(observed[enabled]/quant[enabled])*quant[enabled]
        # Match native stoll(ns)->double * 1e-9 exactly; keep analytic physical time in truth.
        samples.append([float(round(t*1e9))*1e-9,*observed]);truth.append([t,*value])
    return np.array(samples),np.array(truth)

def yaml_config(spec):
    c=spec['camera'];profile=spec['estimator_profile'];noise=profile['config'];params=profile['parameters']
    return ('%YAML:1.0\n---\ndataset_path: .\nframe_skip: 0\nstart_frame: 0\nend_frame: -1\nmodel_type: PINHOLE\ncamera_name: synthetic\n'
            f'image_width: {c["width"]}\nimage_height: {c["height"]}\nprojection_parameters: {{fx: {c["fx"]}, fy: {c["fy"]}, cx: {c["cx"]}, cy: {c["cy"]}}}\n'
            'distortion_parameters: {k1: 0.0, k2: 0.0, p1: 0.0, p2: 0.0}\n'
            f'extrinsicRotation: !!opencv-matrix\n  rows: 3\n  cols: 3\n  dt: d\n  data: {json.dumps(c["r_ic"])}\n'
            f'extrinsicTranslation: !!opencv-matrix\n  rows: 3\n  cols: 1\n  dt: d\n  data: {json.dumps(c["t_ic"])}\n'
            'max_cnt: 120\nmin_dist: 20\nF_threshold: 1.0\nshow_track: 0\nequalize: 1\nfisheye: 0\n'
            'max_solver_time: 10.0\nmax_num_iterations: 10\nkeyframe_parallax: 10.0\n'
            'lk_window_size: 21\nlk_pyramid_levels: 3\nlk_iterations: 20\nlk_eps: 0.03\n'
            f'acc_n: {noise["acc_n"]}\nacc_w: {noise["acc_w"]}\ngyr_n: {noise["gyr_n"]}\ngyr_w: {noise["gyr_w"]}\ng_norm: 9.81\n')

def generate(output,spec):
    output=pathlib.Path(output).resolve()
    if output==pathlib.Path('/mnt/backup/SLAM') or pathlib.Path('/mnt/backup/SLAM') in output.parents:
        raise ValueError('Canonical datasets are read-only')
    output.mkdir(parents=True,exist_ok=False)
    # Exact physical/measurement/estimator configuration is frozen before any rendering or replay.
    write_new(output/'spec.json',json_bytes(spec))
    for directory in ['mav0/cam0/data','mav0/imu0','mav0/mocap0']:(output/directory).mkdir(parents=True,exist_ok=True)
    write_new(output/'config.yaml',yaml_config(spec).encode())
    cam=spec['camera'];profile={'schema':'mobile-slam-camera-profile-v1','pixelFrame':'drawImage','orientationType':'portrait-primary' if cam['height']>cam['width'] else 'landscape-primary',
        'provenance':'Independent synthetic rig; not physical phone calibration',**{k:cam[k] for k in ['width','height','fx','fy','cx','cy','modelType','r_ic','t_ic']},'distortion':[0,0,0,0]}
    write_new(output/'camera-profile.json',json_bytes(profile))
    tex=textures(spec);points=landmarks(spec);imu,imu_truth=sample_imu(spec)
    write_new(output/'landmarks.json',json_bytes(points.tolist()))
    frames=[];truth_frames=[];camera_csv=['#timestamp [ns],filename'];features=['frame,id,x_normalized,y_normalized,z,u,v,vx_normalized,vy_normalized'];body_rows=[];camera_rows=[];gt_csv=['#timestamp [ns],p_W_B_x,p_W_B_y,p_W_B_z,q_w,q_x,q_y,q_z']
    for index in range(spec['frame_count']):
        t=index/spec['image_hz'];state=body_state(t,spec);image,_,_=render(state,spec,tex)
        filename=f'{index:05d}.pgm';payload=image.tobytes();pgm=f'P5\n{cam["width"]} {cam["height"]}\n255\n'.encode()+payload
        write_new(output/'mav0/cam0/data'/filename,pgm)
        frames.append({'index':index,'timestamp':float(round(t*1e9))*1e-9,'physicalTimestamp':t,'file':filename,'sha256':sha(payload)})
        camera_csv.append(f'{round(t*1e9)},{filename}')
        r,p=camera_pose(state,spec);bq=Rotation.from_matrix(state['rotation']).as_quat();cq=Rotation.from_matrix(r).as_quat()
        body={'position':state['position'].tolist(),'rotation':state['rotation'].ravel().tolist(),'quaternion_xyzw':bq.tolist()}
        camera={'position':p.tolist(),'rotation':r.ravel().tolist(),'quaternion_xyzw':cq.tolist()}
        truth_frames.append({'index':index,'timestamp':t,'body':body,'camera':camera,'velocity':state['velocity'].tolist(),'acceleration':state['acceleration'].tolist(),'angularVelocity':state['gyro'].tolist()})
        body_rows.append(' '.join(format(x,'.17g') for x in [t,*state['position'],*bq]));camera_rows.append(' '.join(format(x,'.17g') for x in [t,*p,*cq]))
        gt_csv.append(','.join([str(round(t*1e9)),*[format(x,'.17g') for x in [*state['position'],bq[3],*bq[:3]]]]))
        for feature_id,values in visible_features(points,state,spec,tex):features.append(','.join([str(index),str(feature_id),*[format(x,'.17g') for x in values]]))
        if (index+1)%100==0:print(f'Rendered {index+1}/{spec["frame_count"]}',file=sys.stderr,flush=True)
    imu_csv=['#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z']
    for row in imu:imu_csv.append(','.join([str(round(row[0]*1e9)),*[format(x,'.17g') for x in [*row[4:],*row[1:4]]]]))
    for name,rows in [('mav0/cam0/data.csv',camera_csv),('mav0/imu0/data.csv',imu_csv),('mav0/mocap0/data.csv',gt_csv),('features.csv',features),('trajectory-body-gt.txt',body_rows),('trajectory-camera-gt.txt',camera_rows)]:write_new(output/name,('\n'.join(rows)+'\n').encode())
    write_new(output/'groundtruth.json',json_bytes({'schema':'mobile-slam-synthetic-groundtruth-v1','frames':truth_frames}))
    write_new(output/'imu-truth.json',json_bytes(imu_truth.tolist()))
    input_data={'schema':'mobile-slam-synthetic-vio-v1','scope':'analytic physical motion; synthetic rendered pixels; no estimator truth leakage',
        'spec_sha256':sha((output/'spec.json').read_bytes()),'config':spec['estimator_profile']['config'],'parameters':spec['estimator_profile']['parameters'],
        'frames':frames,'imu':imu.tolist()}
    write_new(output/'input.json',json_bytes(input_data))
    files={str(p.relative_to(output)):{'bytes':p.stat().st_size,'sha256':sha(p.read_bytes())} for p in sorted(output.rglob('*')) if p.is_file()}
    manifest={'schema':'mobile-slam-synthetic-manifest-v1','generator_version':VERSION,'seed':spec['seed'],'scenario':spec['scenario'],
        'spec_sha256':files['spec.json']['sha256'],'input':'input.json','groundtruth':'groundtruth.json','frame_count':len(frames),'imu_count':len(imu),
        'physical_time_span_s':[frames[0]['timestamp'],frames[-1]['timestamp']], 'files':files}
    write_new(output/'manifest.json',json_bytes(manifest))
    return manifest

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=pathlib.Path);parser.add_argument('--seed',type=int,default=101)
    parser.add_argument('--scenario',choices=SCENARIOS,default='clean');parser.add_argument('--portrait',action='store_true')
    args=parser.parse_args(argv);manifest=generate(args.output,make_spec(args.seed,args.scenario,args.portrait))
    print(json.dumps({'output':str(args.output.resolve()),'frames':manifest['frame_count'],'spec_sha256':manifest['spec_sha256']}))

if __name__=='__main__':main()
