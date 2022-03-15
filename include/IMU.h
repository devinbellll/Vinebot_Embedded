#ifndef IMU_H
#define IMU_H

#include "common.h"

#include <ICM_20948.h>

typedef enum {
    W = 0,
    X,
    Y,
    Z
} quat_e;

class IMU {
    public:

        IMU();

        int init(TwoWire *i2c, int AD0_val);

        int read_data(float quat_data);

    private:

        ICM_20948_I2C _ICM_20948;

};

#endif // IMU_H