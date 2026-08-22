#include "MvCameraControl.h"

#include <cstdio>
#include <cstring>

static void printFloat(const char * name, void * handle)
{
  MVCC_FLOATVALUE v;
  std::memset(&v, 0, sizeof(v));
  int ret = MV_CC_GetFloatValue(handle, name, &v);
  if (ret == MV_OK) {
    std::printf("%s: cur=%.3f min=%.3f max=%.3f\n", name, v.fCurValue, v.fMin, v.fMax);
  } else {
    std::printf("%s: query fail nRet [0x%X]\n", name, ret);
  }
}

int main()
{
  int nRet = MV_CC_Initialize();
  if (nRet != MV_OK) return 1;

  auto parseIp = [](const char * ip) {
    int a, b, c, d;
    sscanf(ip, "%d.%d.%d.%d", &a, &b, &c, &d);
    return static_cast<unsigned int>((a << 24) | (b << 16) | (c << 8) | d);
  };

  MV_CC_DEVICE_INFO stDevInfo;
  MV_GIGE_DEVICE_INFO stGigEDev;
  std::memset(&stDevInfo, 0, sizeof(stDevInfo));
  std::memset(&stGigEDev, 0, sizeof(stGigEDev));
  stGigEDev.nCurrentIp = parseIp("192.168.10.10");
  stGigEDev.nNetExport = parseIp("192.168.10.25");
  stDevInfo.nTLayerType = MV_GIGE_DEVICE;
  stDevInfo.SpecialInfo.stGigEInfo = stGigEDev;

  void * handle = nullptr;
  nRet = MV_CC_CreateHandle(&handle, &stDevInfo);
  if (nRet != MV_OK) return 1;
  nRet = MV_CC_OpenDevice(handle);
  if (nRet != MV_OK) {
    std::printf("OpenDevice fail [0x%X]\n", nRet);
    return 1;
  }
  printFloat("ExposureTime", handle);
  printFloat("Gain", handle);
  printFloat("AcquisitionFrameRate", handle);
  printFloat("ResultingFrameRate", handle);
  printFloat("DeviceLinkThroughputLimit", handle);
  MVCC_ENUMVALUE ev;
  std::memset(&ev, 0, sizeof(ev));
  int br = MV_CC_GetEnumValue(handle, "PixelFormat", &ev);
  std::printf("PixelFormat: cur=%u [0x%X]\n", ev.nCurValue, br);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);
  MV_CC_Finalize();
  return 0;
}
