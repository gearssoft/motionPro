#%%
import cv2 as cv
import numpy as np
# import pcl
# import open3d as o3d

from PIL import Image
from IPython.display import display
import yaml
import os

import time
from struct import *

import threading
from rs2_device_manager import Rs2DeviceManagers
#%%
dm = Rs2DeviceManagers()

#%%
_device = dm.get_device_manager(0)
_device.start()

#%%
depth_frame,color_frame = _device.get_frames()

#print data type
print(f"depth_frame: {type(depth_frame)}")
print(f"depth_frame: {type(depth_frame.get_data())}")


#%%
color_image = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())
depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

display( Image.fromarray( cv.cvtColor(color_image,cv.COLOR_BGR2RGB) ) )
display( Image.fromarray( cv.cvtColor(depth_colormap,cv.COLOR_BGR2RGB) ) )


#%%
pointcloud,color_image = _device.getPointCloud(8)
print(color_image.shape)

print(f"len of pointcloud: {len(pointcloud)}")

display( Image.fromarray( cv.cvtColor(color_image,cv.COLOR_BGR2RGB) ) )


#%%
print(f'len of pointcloud: {len(pointcloud)}')
depth_image = np.asanyarray(pointcloud)
depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
display(Image.fromarray(color_image))
display( Image.fromarray( cv.cvtColor(depth_colormap,cv.COLOR_BGR2RGB) ) )


#%%
print(pointcloud[0])
_bb = pointcloud[0].tobytes()
print(f'len of pointcloud: {len(_bb)}')
_bytepc = pointcloud.tobytes()

print(f'len of pointcloud: {len(_bytepc)}')


#%%
_device = dm.get_device_manager(0)
_device.stop()

# %%
