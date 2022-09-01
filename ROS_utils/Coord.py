# import rospy
import numpy as np

# import tf2_ros
import tf.transformations as tf_t
# from geometry_msgs.msg import Transform, Pose
# from Utils import *


class Coord:
    """
    A handy class to do coordinate arithmetic with. Usefully converts poses, transforms, and matrices. Only works in python2 due to dependence on tf.transformations
    """

    # I = Coord(np.eye(4)) # the identity coordinate
    def __init__(self, *inputs):
        if len(inputs) == 2:
            # input is two vectors
            position = inputs[0]
            orientation = inputs[1]

            # cleanse position input
            if len( position ) != 3:
                position = list(position)
                position.append(0)

            P = translation_matrix( position )

            # accept orientation as euler angles or quaternion
            if len(orientation) == 4:
                M = tf_t.quaternion_matrix( orientation )
            else:
                M = tf_t.euler_matrix( orientation[0], orientation[1], orientation[2], 'rzyx' )
            
            self.T = P.dot(M)
            
        elif isinstance(inputs[0], np.ndarray):
            # input is a matrix
            # TODO: input is lone position vector
            self.T = inputs[0]
        else:
            # input is a ROS message
            x,q = search_recursive(inputs[0], [['translation','position'],['rotation','orientation']])

            P = translation_matrix( Vector.to_array( x ))
            M = tf_t.quaternion_matrix( Vector.to_array( q ))
            
            self.T = P.dot(M)

    def to_tf(self):
        tf = Transform()
        x = self.get_translation()
        q = self.get_orientation()
        
        tf.translation = Vector(x)
        tf.rotation = Vector(q)

        return tf

    def to_pose(self):
        pose = Pose()
        x = self.get_translation()
        q = self.get_orientation()
        
        pose.position = Vector(x)
        pose.orientation = Vector(q)
        
        return pose

    def get_translation(self):
        _,_,_,x,_ = tf_t.decompose_matrix(self.T)
        return x

    def get_orientation(self):
        return tf_t.quaternion_from_matrix(self.T)

    def __neg__(self):
        return Coord( np.linalg.inv(self.T) )

    def __add__(self, other):
        return Coord(self.T.dot(other.T))
    def __mul__(self,other):
        return self + other 

    def __sub__(self, other):
        return self + (- other)
    def __div__(self,other):
        return self - other

    def __iadd__(self,other):
        self.T = (self+other).T
    def __imul__(self,other):
        self += other
    
    def __isub__(self,other):
        self.T = (self-other).T
    def __idiv__(self,other):
        self -= other

    def __abs__(self):
        return np.linalg.norm(self.get_translation())

    def __repr__(self):
        return self.T.__str__()


def quaternion_from_matrix(T):
    pass

def translation_from_matrix(T):
    pass

def translation_matrix(x):
    T = np.eye(4)    
    T[0:3,3] = x

    return T

def quaternion_matrix(q):
    # Quaternion is in form .wxyz

    T = np.eye(4)

    # Extract the values from Q
    w, x, y, z = q

    wx,wy,wz, xx,xy,xz = np.ravel( np.matmul( [[w],[x]], [[x,y,z]] ) )
    yy,yz = y* np.array([y,z])
    zz = z*z
    
    s = 2./(w*w + xx + yy + zz)

    T[0:3,0:3] += s * np.array([[ -yy-zz,  xy-wz,  xz+wy ],
                                [  xy+wz, -xx-zz,  yz-wx ],
                                [  xz-wy,  yz+wx, -xx+yy ]])

    return T



def euler_matrix(euler):
    R = quaternion_matrix( quaternion_around_axis(euler[0], [1,0,0]) )
    # print(type(R))
    P = quaternion_matrix( quaternion_around_axis(euler[1], [0,1,0]) )
    Y = quaternion_matrix( quaternion_around_axis(euler[2], [0,0,1]) )

    return Y.dot(P).dot(R)

def quaternion_around_axis(theta, axis):
    ''' Quaternion for rotation of angle `theta` around axis `axis`
    Formula from http://mathworld.wolfram.com/EulerParameters.html
    '''


    axis = np.array(axis)/np.linalg.norm(axis)
    t2 = theta / 2.

    q = np.zeros(4)

    q[0] = np.cos(t2)
    q[1:4] = np.sin(t2) * axis

    return q




theta = 234.0
axis = [342.1234, 43.2, -123.]
axis = np.array(axis)/np.linalg.norm(axis)

q1 = quaternion_around_axis(theta, axis)
q2 = tf_t.quaternion_about_axis(theta, axis)

print(q1[0] == q2[3])
print(q1[1:4] == q2[0:3])