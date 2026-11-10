"""Explicit corrected depth/TF contracts, plus a bounded legacy byte audit.

The original publisher had no TF names or acquisition timestamps. These helpers
never create frame names or camera extrinsics on behalf of a caller.
"""
import base64, math
import numpy as np

def header(seq,stamp_ns,frame_id):
    if not isinstance(stamp_ns,int) or stamp_ns<0:raise ValueError('Explicit nonnegative acquisition time required')
    return {'seq':int(seq),'stamp':{'secs':stamp_ns//1000000000,'nsecs':stamp_ns%1000000000},'frame_id':frame_id}

def depth_image(depth_m,seq,stamp_ns,frame_id,encoding='16UC1'):
    depth=np.asarray(depth_m,dtype=np.float64)
    if depth.ndim!=2:raise ValueError('Depth must be H x W along the optical Z axis, in metres')
    if encoding=='16UC1':
        valid=np.isfinite(depth)&(depth>0)&(depth<=65.535)
        data=np.zeros(depth.shape,dtype='<u2')
        data[valid]=np.rint(depth[valid]*1000).astype('<u2')
        step=depth.shape[1]*2
    elif encoding=='32FC1':
        data=np.where(np.isfinite(depth)&(depth>0),depth,np.nan).astype('<f4')
        step=depth.shape[1]*4
    else:raise ValueError('Supported explicit depth encodings: 16UC1 / 32FC1')
    return {'header':header(seq,stamp_ns,frame_id),'height':depth.shape[0],'width':depth.shape[1],
            'encoding':encoding,'is_bigendian':0,'step':step,'data':base64.b64encode(data.tobytes()).decode('ascii')}

def legacy_depth_bytes(depth_cm):
    """Reproduce Camera.cpp bytes only where the uint16 C++ cast is defined.

    Source multiplies cm by 100, writes the high byte first, but advertises LE.
    Values outside uint16 range are deliberately rejected, not silently wrapped.
    """
    d=np.asarray(depth_cm,dtype=np.float32)
    values=np.floor(d*np.float32(100))
    if np.any(~np.isfinite(values)) or np.any(values<0) or np.any(values>65535):
        raise ValueError('Original float-to-uint16 cast has no portable result for these samples')
    return values.astype('>u2').tobytes()

def tf_message(translation_m,quaternion_xyzw,parent_frame_id,optical_frame_id,seq,stamp_ns):
    if not parent_frame_id or not optical_frame_id or parent_frame_id==optical_frame_id:
        raise ValueError('Original frame names are missing: configure distinct parent and optical frame IDs')
    if parent_frame_id.startswith('/') or optical_frame_id.startswith('/'):
        raise ValueError('TF frame IDs must not have leading slashes')
    values=[*translation_m,*quaternion_xyzw]
    if len(translation_m)!=3 or len(quaternion_xyzw)!=4 or not all(math.isfinite(x) for x in values):
        raise ValueError('Finite translation and quaternion required')
    if abs(sum(x*x for x in quaternion_xyzw)-1)>1e-6:raise ValueError('Unit quaternion required')
    return {'transforms':[{'header':header(seq,stamp_ns,parent_frame_id),'child_frame_id':optical_frame_id,
            'transform':{'translation':dict(zip('xyz',map(float,translation_m))),
                         'rotation':dict(zip('xyzw',map(float,quaternion_xyzw)))}}]}

def verify_contract():
    # Known metric distances, edge limits and invalid pixels, independent of renderer.
    source=np.array([[1.,2.,.1234,65.535],[0.,-1.,np.nan,np.inf]],dtype=float)
    message=depth_image(source,7,1234567890123,'fixture_camera_optical')
    raw=base64.b64decode(message['data'])
    decoded=np.frombuffer(raw,dtype='<u2').reshape(2,4)
    assert decoded.tolist()==[[1000,2000,123,65535],[0,0,0,0]]
    assert raw[:4]==b'\xe8\x03\xd0\x07'
    legacy=legacy_depth_bytes([100.,200.])
    assert legacy==b'\x27\x10\x4e\x20'
    assert np.frombuffer(legacy,dtype='<u2').tolist()==[4135,8270]
    missing_frame_rejected=False
    try:tf_message([0,0,0],[0,0,0,1],None,None,0,0)
    except ValueError:missing_frame_rejected=True
    assert missing_frame_rejected
    return {'passed':True,'correct_1m_bytes_hex':raw[:2].hex(),'legacy_100cm_bytes_hex':legacy[:2].hex(),
            'legacy_100cm_decoded_as_advertised_mm':4135,'corrected_mm_values':decoded.tolist(),
            'stamp':message['header']['stamp'],'missing_tf_frames_rejected':True,
            'scope':'Known numerical fixture; fixes source unit/byte-order bugs explicitly; not a recording from UE ROS'}
