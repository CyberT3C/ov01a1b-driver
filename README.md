
<!-- ABOUT THE PROJECT -->
## About The Project

I want to use my IR cam on dell xps 9315 to use face recognition.
I could not find any driver for this sensor, so I will try to develope one myself

-> https://github.com/NixOS/nixpkgs/issues/225743#issuecomment-3047489839


## Status and Hope 
UPDATE: I have a fully working setup. Every puzzle piece is coming togehter right now and I dont need to guess any more and can debug the exact problems.

- [X] v0.1 alpha driver which will register a v4l2 media device
- [X] (Missing in Firmware) ACPI Graph for our sensor
- [X] IPU-Bridge patch, so the IPU does know our sensor and will connect.
- [X] Media Device is listing the format I setup :)
- [X] Realtime Debug Data from IPU6 is our images are getting processed - Hell yeah!

``` bash  
# IPU6 is sending Data on request
We tried a format Y10 first which is common for IR sensors but this failed.
 v4l2-ctl -d /dev/video8 --set-fmt-video=width=1280,height=800,pixelformat='Y10 ' --stream-mmap --stream-count=1 --stream-to=test.raw
The pixelformat 'Y10 ' is invalid

# dmesg
[ 1117.197533] intel_ipu6_isys.isys intel_ipu6.isys.40: buffer: Intel IPU6 ISYS Capture 8: configured size 4151040, buffer size 4151040
[ 1117.197555] intel_ipu6_isys.isys intel_ipu6.isys.40: buffer: Intel IPU6 ISYS Capture 8: configured size 4151040, buffer size 4151040
[ 1117.197561] intel_ipu6_isys.isys intel_ipu6.isys.40: buffer: Intel IPU6 ISYS Capture 8: configured size 4151040, buffer size 4151040
[ 1117.197565] intel_ipu6_isys.isys intel_ipu6.isys.40: buffer: Intel IPU6 ISYS Capture 8: configured size 4151040, buffer size 4151040
[ 1117.197576] intel_ipu6_isys.isys intel_ipu6.isys.40: queue buffer 0 for Intel IPU6 ISYS Capture 8
[ 1117.197581] intel_ipu6_isys.isys intel_ipu6.isys.40: iova: iova 0x000000007f383000
[ 1117.197586] intel_ipu6_isys.isys intel_ipu6.isys.40: media pipeline is not ready for Intel IPU6 ISYS Capture 8
[ 1117.197589] intel_ipu6_isys.isys intel_ipu6.isys.40: queue buffer 1 for Intel IPU6 ISYS Capture 8
[ 1117.197592] intel_ipu6_isys.isys intel_ipu6.isys.40: iova: iova 0x000000007ef8d000
[ 1117.197594] intel_ipu6_isys.isys intel_ipu6.isys.40: media pipeline is not ready for Intel IPU6 ISYS Capture 8
[ 1117.197597] intel_ipu6_isys.isys intel_ipu6.isys.40: queue buffer 2 for Intel IPU6 ISYS Capture 8
[ 1117.197600] intel_ipu6_isys.isys intel_ipu6.isys.40: iova: iova 0x000000007eb97000
[ 1117.197602] intel_ipu6_isys.isys intel_ipu6.isys.40: media pipeline is not ready for Intel IPU6 ISYS Capture 8
[ 1117.197605] intel_ipu6_isys.isys intel_ipu6.isys.40: queue buffer 3 for Intel IPU6 ISYS Capture 8
[ 1117.197607] intel_ipu6_isys.isys intel_ipu6.isys.40: iova: iova 0x000000007e7a1000
[ 1117.197609] intel_ipu6_isys.isys intel_ipu6.isys.40: media pipeline is not ready for Intel IPU6 ISYS Capture 8
[ 1117.197613] intel_ipu6_isys.isys intel_ipu6.isys.40: stream: Intel IPU6 ISYS Capture 8: width 1920, height 1080, css pixelformat 24
[ 1117.197635] intel_ipu6_isys.isys intel_ipu6.isys.40: validating link "Intel IPU6 CSI2 1":1 -> "Intel IPU6 ISYS Capture 8"
[ 1117.197640] intel_ipu6_isys.isys intel_ipu6.isys.40: format mismatch 4096x3072,300a != 1920x1080,3008
[ 1117.197645] intel_ipu6_isys.isys intel_ipu6.isys.40: media pipeline start failed
[ 1117.197648] intel_ipu6_isys.isys intel_ipu6.isys.40: failed to setup video
```


First milestone hit /analyze/power_sequence_summary.txt

looks like we can use the same methods as 
- driver from linx kernel ov01a10
- "" ov01a1s (from my understanding is that a module which combines then sensors ov01a10 and ov01a1b in a single one)

If all my assumptions are true, there will be a ready to use driver in the future.

## Knowledge
? ?  
  
┌─────────────────┐  
│    PipeWire     │  <- I want this   
├─────────────────┤  
│    libcamera    │  <- use that API  
├─────────────────┤  
│  V4L2 Userspace │  <- /dev/videoX  
╞═════════════════╡ ← Kernel/User   
│  V4L2 Subsystem │  <- Will integrate my driver and there is no other Video Stack in the linux kernel?  
├─────────────────┤  
│  this IR Driver │  <- ov01a1b.c  
├─────────────────┤  
│   I2C/Hardware  │  
└─────────────────┘  
  
Due to V4L2 Subsystem seems like rust ist no choice for my driver  
  
Power On/Off works, whats next?  
What is my aim?
Defined Goal:

Boot:
  └── probe (power on → check ID → power off → register)

Cam App startup:
  └── open() → pm_runtime_get() → runtime_resume → power_on

Stream start:
  └── VIDIOC_STREAMON → s_stream(1) → sensor config

Stream stop:
  └── VIDIOC_STREAMOFF → s_stream(0)

Cam App close:
  └── close() → pm_runtime_put() → runtime_suspend → power_off


at some point in the future ->
# OV01A1B Linux Driver

Linux V4L2 driver for OmniVision OV01A1B camera sensor.
[STATUS] under construction!

## License
GPL-2.0 (see LICENSE file)

## Credits
Based on the ov01a10 driver by Intel Corporation.
