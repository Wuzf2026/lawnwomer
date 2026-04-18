# HesaiLidar_SDK_2.0
## About the project
This repository includes the software development kit for Hesai LiDAR sensor manufactured by Hesai Technology

## Support Lidar type
- JT128

## Environment and Dependencies

**System environment requirement:Linux**
```
Recommanded
-Ubuntu 16.04
-Ubuntu 18.04
-Ubuntu 20.04
-Ubuntu 22.04
-Windows 10
```

**Compiler vresion requirement**
```
Cmake version requirement:Cmake 3.8.0 or above
G++ version requirement:G++ 7.5 or above
```
**Library Dependencies: libpcl-dev + libpcap-dev + libyaml-cpp-dev
```
$ sudo apt install libpcl-dev libpcap-dev libyaml-cpp-dev
```

## Clone
```
$ git clone https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0.git

Note: Window is not recommended to use the compressed package, there will be symbol problems lead to compilation errors
```

## Ubuntu Build
```
1.$ cd HesaiLidar_SDK_2.0
2.$ mkdir build
3.$ cd build
4.$ cmake ..
5.$ make
```

## Ubuntu Build In Gpu
'''
1.$ cd HesaiLidar_SDK_2.0
2.$ mkdir build
3.$ cd build
4.$ cmake .. -DFIND_CUDA=1
5.$ make
'''

## Window Build
```
Environmental preparations:
	- Visual Studio2022	 https://visualstudio.microsoft.com/zh-hans/downloads/
	- cmake-gui  		 https://cmake.org/download/
	- Git 				 https://git-scm.com/

Compile Environment Configuration:
	1. Open CMake-GUI, select the source directory (`HesaiLidar_SDK_2.0`) and output directory (`HesaiLidar_SDK_2.0/build`)
	2. Click `Configure`
  	3. Click `Generate`
	4. If it prints “Configuring done” and “Generating done” then it is OK, otherwise check for errors.
	5. Click on `Open Project`
	6. Right click `ALL BUILD` and click `Generate`
	7. The corresponding executable file can be found in the `Debug/Release` folder under `Build`
```

## Run a sample

Set the parameters in param in main.cc or main.cu
```
// Reciving data from pcap file
```
	param.use_gpu = false;
	param.input_param.source_type = DATA_FROM_PCAP;
	param.input_param.pcap_path = "Your pcap file path";
	param.input_param.correction_file_path = "Your correction file path";
	param.input_param.firetimes_path = "Your firetime file path";
```
// Reciving data from connected Lidar
```
	param.use_gpu = false;
	param.input_param.source_type = DATA_FROM_LIDAR;
	param.input_param.device_ip_address = "192.168.1.201";
	param.input_param.udp_port = 2368;
	param.input_param.ptc_port = 9347;
	param.input_param.correction_file_path = "Your correction file path";
	param.input_param.firetimes_path = "Your firetime file path";
```

$ make 
// run a cpu sample
$ ./sample
// run a gpu sample
$ ./sample (when param.use_gpu = true)
```

## Functional Parameter Description
```
DecoderParam :
	1. pcap_play_synchronization: When parsing pcap, it is delayed according to the point cloud time to simulate the publish speed when parsing in real time.
	2. frame_start_azimuth: Split-frame position of the 360-degree rotating lidar (in degrees [0-360)).
	3. use_timestamp_type: Use timestamp type (point cloud carry or local time).
	4. fov_start and fov_end: Allowable light emission angle, outside the angle is set to 0.
	5. distance_correction_lidar_flag: Control of optical center corrections for mechanical lidar.
	6. Setting the buffer size of the system udpsocket.
InputParam:
	1. source_type: udp data sources.
	2. device_ip_address: lidar ip
	3. udp_port: point cloud udp port
	4. ptc_port: lidar ptc port
	5. multicast_ip_address: point cloud udp is multicast
	6. correction_save_path: Serial port to get the storage path of the angle calibration file (only for JT16).
	7. pcap_path: Local path to pcap when pcap parses.
	8. correction_file_path: Local path to correction file.
	9. standby_mode: Initialization sets the lidar mode to * (on if not -1).
	10. speed: Initialization sets the lidar speed to * (on if not -1).
DriverParam: 
	1. log_level: Log level to be output.
	2. log_Target: Log output location, print and file.
	3. log_path: File location for log output.
	4. use_gpu: use cuda to parser
```