// Stand-in for fs-4's vcu/imu/vectornav_imu.h in the parity build. The ETC
// only uses VectornavState, so the VectorNav SDK isn't needed. CMakeLists.txt
// copies this next to the fs-4 copy of etc_controller.h.

#ifndef VECTORNAV_IMU_H
#define VECTORNAV_IMU_H

#include "mbed.h"

struct VectornavState {
    struct Ypr {
        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
    };
    struct Lla {
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
    };

    float accel[3] = {0.0f, 0.0f, 0.0f};
    float ang_rate[3] = {0.0f, 0.0f, 0.0f};
    Ypr ypr;
    Lla pos;
    float vel[3] = {0.0f, 0.0f, 0.0f};
};

#endif
