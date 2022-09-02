import rospy
import numpy as np

"""
A file of handy functions All run in both python 2 and 3.
"""

def search_recursive(object, attributes, max_depth=3):
    """
    Handy function to look for attribute names in an object recursively. Particularly useful when trying to extract `position` or `translation` from many possible types of ROS message
    """


    # reshape attributes to be a 2d array. Allows more general inputs
    for i in range(len(attributes)):
        attributes[i] = np.ravel(attributes[i])

    # define output
    out = [None] * len(attributes)


    # define recursion function
    def search(sub_objects, depth):
        for sub_object in sub_objects:
            if sub_object is None:
                continue

            # search top level
            for i in range(len(attributes)):
                if attributes[i] is None:
                    continue
                
                for attribute in attributes[i]:
                    if hasattr(sub_object, attribute):
                        out[i] = getattr(sub_object, attribute)
            
            # check if done
            if ((np.array(out) != None).all()):
                return

        # stop if depth exceeded
        if depth < max_depth:
            search( [getattr(sub_object, attr, None)
                for sub_object in sub_objects
                for attr in dir(sub_object)],
                depth + 1)
    
    # search recurseively
    search([object], 1)

    return out

def await_condition(condition, timeout=60, on_condition=lambda: 0, on_timeout=lambda: 0, sleep_time=0.01):
    """
    Handy function to delay until a condition is met. Offers optional arguments for on_condition and on_timeout (which will be returned if the condition is met or the function times out). Also allows the timeout duration to be specified
    """

    start_time = rospy.get_rostime().to_sec()
    
    while not rospy.is_shutdown() and rospy.get_rostime().to_sec() - start_time < timeout:
        if condition():
            return on_condition()
            
        rospy.sleep(sleep_time)

    return on_timeout()

def quaternion_from_matrix(T):
    
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = T[:3,:3].flat
    # Fill only lower half of symmetric matrix
    K = np.array([[Qxx - Qyy - Qzz, 0,               0,               0              ],
                  [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
                  [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
                  [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]) / 3.0
    # Use Hermitian eigenvectors, values for speed
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector, reorder to w,x,y,z quaternion
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Prefer quaternion with positive w
    # (q * -1 corresponds to same rotation as q)
    if q[0] < 0:
        q *= -1
    
    return q

def translation_from_matrix(T):
    return np.ravel( T[:3, 3])

def translation_matrix(x):
    T = np.eye(4)    
    T[:3,3] = x

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
    
    T[:3,:3] += s * np.array([[ -yy-zz,  xy-wz,  xz+wy ],
                                 [  xy+wz, -xx-zz,  yz-wx ],
                                 [  xz-wy,  yz+wx, -xx-yy ]])

    return T

def euler_matrix(euler):
    R = quaternion_matrix( quaternion_around_axis(euler[0], [1,0,0]) )
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