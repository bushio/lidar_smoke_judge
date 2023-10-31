from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sys
import time
import array

def point_cloud2_to_array(msg):
    """
    Convert a sensor_msgs/PointCloud2 message to a NumPy array. The fields
    in the PointCloud2 message are mapped to the fields in the NumPy array
    as follows:
    * x, y, z -> X, Y, Z
    * intensity -> I
    """
    for field in msg.fields:
        # float32 (data type=7)
        if field.name == "intensity":
            intensity_offset = field.offset

        # uint16 (data type=4)
        if field.name == "ring":
            ring_offset = field.offset

        # float32 (data type=7)
        if field.name == "time":
            time_offset = field.offset

    # Convert the PointCloud2 message to a NumPy array
    pc_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = pc_data[:, 0:12].copy().view(dtype=np.float32).reshape(-1, 3)
    xyz = np.nan_to_num(xyz)

    distance = np.sqrt(xyz[:,0] ** 2 + xyz[:,1] ** 2 + xyz[:,2] ** 2)

    intensity = pc_data[:, intensity_offset:intensity_offset + 4].copy().view(dtype=np.float32)
    intensity = np.nan_to_num(intensity)
    
    t = pc_data[:, time_offset:time_offset + 4].copy().view(dtype=np.float32)
    
    # Return the arrays in a dictionary
    return {"X": xyz[:, 0], "Y": xyz[:, 1], "Z": xyz[:, 2],  "I": intensity, "Dist": distance}

