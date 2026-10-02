#ifndef VECTORNAV_STATE_H
#define VECTORNAV_STATE_H

// Latest VectorNav VN-200 outputs. Same field names and types as upstream's
// VectornavState, which used the VectorNav SDK's Vec3f, Ypr and Lla.

struct VnYpr {
    float yaw = 0.0f; // deg
    float pitch = 0.0f;
    float roll = 0.0f;
};

struct VnLla {
    double lat = 0.0; // deg
    double lon = 0.0; // deg
    double alt = 0.0; // m
};

struct VectornavState {
    float accel[3] = {0.0f, 0.0f, 0.0f};    // body frame, m/s^2
    float ang_rate[3] = {0.0f, 0.0f, 0.0f}; // body frame, rad/s
    VnYpr ypr;
    VnLla pos;
    float vel[3] = {0.0f, 0.0f, 0.0f}; // body frame, m/s
};

#endif
