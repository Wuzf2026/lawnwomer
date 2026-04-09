/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/
#ifndef Udp1_4_PARSER_GPU_H_
#define Udp1_4_PARSER_GPU_H_
#include <stdint.h>
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include <semaphore.h>
#include <list>
#include <sstream>
#include <vector>
#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

#include "lidar_types.h"
#include "plat_utils.h"
#include "logger.h"
#include "udp_parser_gpu_kernel.h"

#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_device_runtime_api.h>
#include "safe_call.cuh"
#include "return_code.h"
#include "udp1_4_parser.h"
#define HALF_CIRCLE 180.0
#ifndef M_PI
#define M_PI 3.1415926535898
#endif

namespace hesai
{
namespace lidar
{
// class Udp1_4ParserGpu
// computes points for JT16
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp1_4ParserGpu {
 private:
  JT128buffer* point_could_cu_;
  double* correction_azi_cu_;
  double* correction_ele_cu_;
  float* firetime_correction_cu_;
  LidarPointXYZAIW* points_;
  LidarPointXYZAIW* points_cu_;
  const CorrectionData* correction_ptr;
  const float* firetime_ptr;
  bool get_correction_file_;
  bool get_firetime_file_;
  LidarOpticalCenter optical_center;
 public:
  Udp1_4ParserGpu(uint16_t maxPacket, uint16_t maxPoint);
  ~Udp1_4ParserGpu();

  // compute xyzi of points from decoded packet， use gpu device
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  virtual void LoadCorrectionStruct(void *);
  virtual void LoadFiretimesStruct(void *);
};
}
}
#include "udp1_4_parser_gpu.cu"
#endif  // Udp1_4_PARSER_GPU_H_
