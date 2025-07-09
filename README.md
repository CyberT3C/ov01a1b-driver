
<!-- ABOUT THE PROJECT -->
## About The Project

I want to use my IR cam on dell xps 9315 to use face recognition.
I could not find any driver for this sensor, so I will try to develope one myself

-> https://github.com/NixOS/nixpkgs/issues/225743#issuecomment-3047489839


## Status and Hope 
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


