import rospy
import numpy as np

class Vector:
    """
    A handy class to convert between iterables and .xyzw datatypes
    """
    def __init__(self, array):
        self.x = array[0]
        self.y = array[1]
        if len(array) > 2:
            self.z = array[2]
        if len(array) > 3:
            self.w = array[3]
    
    @staticmethod
    def to_array(vector):
        if vector is None:
            return None
            
        if hasattr(vector, 'w'):
            return np.array([vector.x, vector.y, vector.z, vector.w])
        elif hasattr(vector, 'z'):
            return np.array([vector.x, vector.y, vector.z])
        else:
            return np.array([vector.x, vector.y])

    def __repr__(self):
        return Vector.to_array(self).__repr__()