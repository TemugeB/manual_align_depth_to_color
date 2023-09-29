**Manually align depth image to color frame**

![image](https://github.com/TemugeB/manual_align_depth_to_color/assets/36071915/ad811c96-0320-4d22-8e2c-9f20e0d02f57)


This demo shows how to manually align depth image to color frame in the context of a Realsense camera images. If you don't have a realsense camera, the demo here will still be useful to you. You only need to determine the depth and color camera intrinsic parameters and the depth to color camera extrinsic transform.

**Prerequisites**

1. Before you start, plug in your Realsense camera. This demo was only test on D415, at 1280x720 resolution. May not work on non-D cameras.
2. Ensure you have Realsense2 SDK installed with python support. As of writing this demo, the Realsense2 SDK does not support linux Kernel 6+, meaning the binaries must be build from source. You can find my instructions here: [link](https://gist.github.com/TemugeB/f810eeeb022e5c7afb51d09079bcda7d). If you don't have a Realsense camera, then just read the demo code. You will see that the Realsense SDK is only used to obtain the camera intrinsic and extrinsic parameters. 
3. Read and run the depth to color alignment demo provided by Intel: [link](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)

**How-to**

Just copy the provided demo code and run with:

```python3 align_depth_to_color.py```
