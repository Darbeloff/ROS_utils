from unittest import TestCase

import numpy as np

import ros_utils
import tf.transformations as tf_t

class TestGeometry(TestCase):

    def get_quaternion_pair(self):
        axis = np.random.normal(size=3)
        axis = axis / np.linalg.norm(axis)

        theta = np.random.uniform(0, np.pi)

        q1 = ros_utils.quaternion_around_axis(theta, axis)
        q2 = tf_t.quaternion_about_axis(theta, axis)

        return (q1, q2)

    def test_quaternion_around_axis(self):
        for i in range(20):
            q1, q2 = self.get_quaternion_pair()

            self.assertEqual(q1[0], q2[-1])
            self.assertTrue(np.allclose( q1[1:4], q2[0:3]))

    def test_quaternion_matrix(self):
        for i in range(20):
            q1, q2 = self.get_quaternion_pair()
            
            T1 = ros_utils.quaternion_matrix(q1)
            T2 = tf_t.quaternion_matrix(q2)

            self.assertTrue( np.allclose( T1, T2 ) )

    def test_euler_matrix_simple(self):
        euler = []
        M = []

        euler.append( [0,0,0] )
        M.append( np.eye(4) )

        euler.append( [2*np.pi,2*np.pi,2*np.pi] )
        M.append( np.eye(4) )

        # half rotation about z
        euler.append( [0,0,np.pi] )
        M.append( [[ -1, 0, 0, 0],
                   [ 0, -1, 0, 0],
                   [ 0, 0, 1, 0],
                   [ 0, 0, 0, 1]] )

        # half rotation about x
        euler.append( [np.pi,0,0] )
        M.append( [[ 1, 0, 0, 0],
                   [ 0, -1, 0, 0],
                   [ 0, 0, -1, 0],
                   [ 0, 0, 0, 1]] )

        # quarter rotation about x
        euler.append( [np.pi/2.,0,0] )
        M.append( [[ 1, 0, 0, 0],
                   [ 0, 0, -1, 0],
                   [ 0, 1, 0, 0],
                   [ 0, 0, 0, 1]] )

        # quarter rotation about x
        euler.append( [np.pi/2.,0,np.pi] )
        M.append( [[ -1, 0, 0, 0],
                   [ 0, 0, 1, 0],
                   [ 0, 1, 0, 0],
                   [ 0, 0, 0, 1]] )


        for e,m in zip(euler, M):
            result = ros_utils.euler_matrix(e)
            self.assertTrue( np.allclose( result, m ) )

    def test_matrix_from_quaternion(self):
        for i in range(20):
            q, _ = self.get_quaternion_pair()

            T = ros_utils.quaternion_matrix(q)
            q2 = ros_utils.quaternion_from_matrix(T)

            self.assertTrue( np.allclose( q, q2))