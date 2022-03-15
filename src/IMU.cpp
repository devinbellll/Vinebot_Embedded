#include "IMU.h"

IMU::IMU() {}

int IMU::init(TwoWire *i2c, int AD0_val)
{
    bool initialized = false;
    int try_count = 0;
    while (!initialized && try_count <= 5)
    {
        _ICM_20948.begin(*i2c, AD0_val);

        Serial.print("Initialization: ");
        Serial.println(_ICM_20948.statusString());

        if (_ICM_20948.status != ICM_20948_Stat_Ok)
        {
            Serial.println(F("Trying again..."));
            delay(500);
        }        
        else
        {
            initialized = true;
            Serial.println("Connected");
        }
        try_count++;
    }

    if (initialized == false)
    {
        Serial.println("Too many tries, initialization failed");
        return VINEBOT_ERR;
    }

    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (_ICM_20948.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    success &= (_ICM_20948.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

    // Enable any additional sensors / features
    //success &= (_ICM_20948.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (_ICM_20948.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (_ICM_20948.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (_ICM_20948.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (_ICM_20948.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (_ICM_20948.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (_ICM_20948.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (_ICM_20948.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (_ICM_20948.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (_ICM_20948.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (_ICM_20948.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (_ICM_20948.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (_ICM_20948.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
        Serial.println(F("DMP enabled"));
        return VINEBOT_OK;
    }
    else
    {
        Serial.println(F("Enable DMP failed"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        return VINEBOT_ERR;
    }
}

int IMU::read_data(float *quat_data)
{
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t data;
    _ICM_20948.readDMPdataFromFIFO(&data);

    if ((_ICM_20948.status == ICM_20948_Stat_Ok) || (_ICM_20948.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.

            // Scale to +/- 1
            quat_data[X] = ((float)data.Quat9.Data.Q1) / 1073741824.0; // Convert to float. Divide by 2^30
            quat_data[Y] = ((float)data.Quat9.Data.Q2) / 1073741824.0; // Convert to float. Divide by 2^30
            quat_data[Z] = ((float)data.Quat9.Data.Q3) / 1073741824.0; // Convert to float. Divide by 2^30
            quat_data[W] = sqrt(1.0 - ((quat_data[X] * quat_data[X]) + (quat_data[Y] * quat_data[Y]) + (quat_data[Z] * quat_data[Z])));

            return VINEBOT_OK;
        }
        else 
        {
            return VINEBOT_ERR;
        }
    }   
    else 
    {
        return VINEBOT_ERR;
    }
}