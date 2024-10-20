#ifndef ASTRA_DEVICE_TYPE_H
#define ASTRA_DEVICE_TYPE_H

constexpr static unsigned ASTRA_MINI_PID = 0x0404;
constexpr static unsigned STEREO_S_DEPTH_PID = 0x0608;
constexpr static unsigned STEREO_S_U3_DEPTH_PID = 0x0614;

#define OB_STEREO_S "Orbbec Canglong"
#define OB_EMBEDDED_S "Astra SL1000S_U3"
#define OB_ASTRA_PRO "Orbbec Astra Pro"
#define OB_STEREO_S3 "Astra SV1301S_U3"

typedef enum
{
    OB_ASTRA_NO,
    OB_STEREO_S_NO,
    OB_EMBEDDED_S_NO,
    OB_ASTRA_PRO_NO,
    OB_STEREO_S3_NO
} OB_DEVICE_NO;

bool astraWithUVC(OB_DEVICE_NO id);

#endif
