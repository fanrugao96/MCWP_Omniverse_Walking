"""Application-level validation: UI callbacks, UsdSkel, PhysX and actual captures."""
import asyncio,csv,datetime,io,json,math,os,struct,traceback,uuid
from pathlib import Path
import numpy as np
import carb
import omni.kit.app
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from PIL import Image

def sphere_mesh_gap(center,radius,points,faces):
    triangles=points[faces];a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
    ab=b-a;ac=c-a;n=np.cross(ab,ac);n2=np.einsum('ij,ij->i',n,n)
    signed=np.einsum('ij,ij->i',center-a,n)/np.maximum(n2,1e-16)
    projected=center-signed[:,None]*n
    v=projected-a;d00=np.einsum('ij,ij->i',ab,ab);d01=np.einsum('ij,ij->i',ab,ac);d11=np.einsum('ij,ij->i',ac,ac)
    d20=np.einsum('ij,ij->i',v,ab);d21=np.einsum('ij,ij->i',v,ac);den=d00*d11-d01*d01
    beta=(d11*d20-d01*d21)/np.maximum(den,1e-16);gamma=(d00*d21-d01*d20)/np.maximum(den,1e-16)
    inside=(beta>=0)&(gamma>=0)&(beta+gamma<=1)&(n2>1e-14)
    distances=np.where(inside,np.abs(signed)*np.sqrt(n2),np.inf)
    for start,end in [(a,b),(b,c),(c,a)]:
        edge=end-start;t=np.clip(np.einsum('ij,ij->i',center-start,edge)/np.maximum(np.einsum('ij,ij->i',edge,edge),1e-16),0,1)
        distances=np.minimum(distances,np.linalg.norm(start+t[:,None]*edge-center,axis=1))
    return float(distances.min()-radius)

async def verify(w):
    from worker_full_validation import verify as current_verify
    return await current_verify(w)

async def record_detail(w):
    r=w.r;out=Path(__file__).resolve().parents[1]/'evidence'
    if r.busy:return
    r.reset();r.busy=True;r.running=False;w.view(True)
    result={'file':'evidence/worker_closeup.avi','fps':30,'actual_kit_capture':True}
    try:
        await r.apply_capture_exposure()
        for _ in range(8):await omni.kit.app.get_app().next_update_async()
        folder=out/'worker_detail_frames'/uuid.uuid4().hex;folder.mkdir(parents=True)
        sequence=[('idle',.6),('forward',2),('release',.7),('back',2),('release',.7),('left',1.5),('release',.5),('right',1.5),('release',.5),('arms_up',1.8),('arms_down',1.8)]
        frames=[]
        for label,seconds in sequence:
            w.stop()
            if label in ['forward','back','left','right']:w.press(label,source='keyboard')
            if label in ['arms_up','arms_down']:w.pose_command(label=='arms_up')
            for i in range(round(seconds*30)):
                for _ in range(2):w.step(1/60);r.step(1/60)
                w.update_camera();await omni.kit.app.get_app().next_update_async()
                path=folder/f'{len(frames):04d}.png';await r.capture_image(path)
                with Image.open(path) as im:
                    buf=io.BytesIO();im.convert('RGB').save(buf,format='JPEG',quality=90);frames.append(buf.getvalue())
                if i==round(seconds*30)-1:(out/f'worker_closeup_{label}.png').write_bytes(path.read_bytes())
                if len(frames)%90==0:print(f'WORKER_CLOSEUP {len(frames)} frames',flush=True)
        r.write_video(out/'worker_closeup.avi',frames,fps=30)
        result.update(frames=len(frames),duration_s=len(frames)/30,complete=True)
    except Exception:
        result.update(complete=False,error=traceback.format_exc());print(result['error'])
    finally:
        (out/'worker_closeup_validation.json').write_text(json.dumps(result,indent=2))
        r.busy=False;r.reset();r.restore_preview_exposure();r.message('Worker close-up preview complete.' if result.get('complete') else 'Close-up capture failed; see report.')
    return result
