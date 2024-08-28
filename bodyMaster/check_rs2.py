#%%
from horse_house.motionPro.bodyMaster.deviceManager_rs2 import Rs2DeviceManagers , __VERSION__ as rs2_device_manager_version
#%%
dm = Rs2DeviceManagers()

# print(f"Rs2DeviceManagers version: {rs2_device_manager_version}")


# %%

for device in dm.device_managers:
    print(device)
    print(device.SerialNumber)
    print(device.ProductLine)
    print(device.cameraName)
    print(device.FirmwareVersion)
    

# %%
