#!/usr/bin/env python3
import threading
import time
import unittest

import numpy as np
import rospy
import rostest
from mid360_ray_msgs.msg import CheckedRayBundle, Ray
from sensor_msgs.msg import PointCloud2, PointField


class MaskBurstTest(unittest.TestCase):
    def test_mask_preserves_rays_and_burst_frames(self):
        lock=threading.Lock(); received={}; counts={}
        def rays(m):
            # Store compact results, not all 131k ROS objects per frame.
            with lock:
                received[m.header.seq]=(len(m.rays),sum(r.source.return_status==Ray.INVALID_RANGE for r in m.rays),
                                        sum(r.source.return_status==Ray.NO_RETURN for r in m.rays),
                                        all(r.original_index==i for i,r in enumerate(m.rays)))
        def points(m):
            with lock: counts[m.header.seq]=m.width*m.height
        sub=rospy.Subscriber('/uav1/ouster/rays_checked',CheckedRayBundle,rays,queue_size=32,buff_size=64*1024*1024)
        sub2=rospy.Subscriber('/uav1/ouster/points_world',PointCloud2,points,queue_size=32,buff_size=32*1024*1024)
        pub=rospy.Publisher('/test/raw_os1',PointCloud2,queue_size=32)
        deadline=time.monotonic()+15
        while pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.05)
        self.assertGreater(pub.get_num_connections(),0)
        time.sleep(.5)
        n=1024*128
        data=np.zeros(n,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('range','<u4'),('ring','u1'),('t','<u4')])
        az=np.tile(np.linspace(0,2*np.pi,1024),128);el=np.repeat(np.linspace(-np.pi/8,np.pi/8,128),1024)
        data['x']=5*np.cos(el)*np.cos(az);data['y']=5*np.cos(el)*np.sin(az);data['z']=5*np.sin(el)
        data['range']=5000;data['ring']=np.repeat(np.arange(128),1024)
        fields=[PointField(name=k,offset=data.dtype.fields[k][1],datatype=(7 if k in ['x','y','z','intensity'] else 2 if k=='ring' else 6),count=1) for k in data.dtype.names]
        expected=set(range(8))
        for seq in expected:
            m=PointCloud2();m.header.seq=seq;m.header.stamp=rospy.Time.from_sec(10+.004*seq);m.header.frame_id='uav1/os_sensor'
            m.height=128;m.width=1024;m.fields=fields;m.point_step=data.dtype.itemsize;m.row_step=1024*m.point_step;m.data=data.tobytes()
            pub.publish(m);time.sleep(.004)
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            with lock:
                if set(received)==expected and set(counts)==expected:break
            time.sleep(.1)
        self.assertEqual(set(received),expected);self.assertEqual(set(counts),expected)
        for seq,(nr,blocked,no_return,identity_ok) in received.items():
            self.assertEqual(nr,n);self.assertGreater(blocked,0);self.assertEqual(no_return,0)
            self.assertEqual(counts[seq],n-blocked);self.assertTrue(identity_ok)
        sub.unregister();sub2.unregister()


if __name__=='__main__':
    rospy.init_node('ouster_mask_burst_test')
    rostest.rosrun('mid360_ray_preprocessor','ouster_mask_burst',MaskBurstTest)
