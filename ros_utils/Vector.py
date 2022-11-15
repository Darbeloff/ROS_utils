import rospy
import numpy as np

class Vector:
    """
    A handy class to convert between iterables and .wxyz datatypes
    """
    def __init__(self, array):
        if len(array) > 3:
            self.w = array[0]
            w_offset = 1
        else:
            w_offset = 0
        
        self.x = array[0 + w_offset]
        self.y = array[1 + w_offset]
        if len(array) > 2:
            self.z = array[2 + w_offset]
    
    @staticmethod
    def to_array(vector):
        if vector is None:
            return None
            
        if hasattr(vector, 'w'):
            # Quaternions are .wxyz
            return np.array([vector.w, vector.x, vector.y, vector.z])
        elif hasattr(vector, 'z'):
            return np.array([vector.x, vector.y, vector.z])
        else:
            return np.array([vector.x, vector.y])

    def __repr__(self):
        return Vector.to_array(self).__repr__()