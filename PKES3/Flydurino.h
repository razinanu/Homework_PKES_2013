
// ---------------------------------------------------------------------------
// Receive all measurements from an 9 DOF sensor board. 
//
// Sebastian Zug
// Otto-von-Guericke University, Magdeburg, Germany
// 09/2013
// ---------------------------------------------------------------------------
//
// Structure:
//                                         
//                                           Sub I2C
//                                        ______^ ______
//                                       |              |
// ----------
//  Arduino  |               ----------------          -------------
//  2560     |- 3.3 V ------ | MPU 6050     |          |  HMC5883  |
//           |- GND ---------| Acceleration,|---SDA ---|  Compass  |
//           |- SDA ---------| Gyro, Temp   |---SCL ---|           |
//           |- SCL ---------|              |          |           |
//           |               ----------------          -------------
//-----------
//                       |___________________ _______________________|
//                                           V
//                                 Integrated IMU sensor offered
//
// Pull-up resistors are integrated in the sensor board.
//
// IMPORTANT: When I connect the sensor board to a 5V power supply, it was 
//            not possible to realize a I2C connection in this case. I made
//            some experiments with additional pull-upps on the I2C but 
//            without any results.
//
// ---------------------------------------------------------------------------
//
// It exists a very good library for I2C communication based on Arduino "Wire"
// provided by Jeff Rowberg. It integrates specific controllers as MPU 6050 
// and HMC 5883. Take a view on https://github.com/jrowberg/i2cdevlib
//
// The example was implement with i2cdevlib Version and extends the existing 
// MPU_6050_raw // example. It uses the code proposed by @muzhig on i2cdevlib 
// https://github.com/jrowberg/i2cdevlib/issues/18
// ---------------------------------------------------------------------------

#include "MPU6050.h"
#include "HMC5883L.h"

class Flydurino : public MPU6050, public HMC5883L {
  private:    
    MPU6050 mpu6050;
    HMC5883L hmc5883l;
    int8_t mpu6050Connected;
    int8_t hmc5883lConnected;
  public:
    Flydurino();
    void setSlaveControl(uint8_t slaveID);
    int8_t getmpu6050State();
    int8_t gethmc5883lState();
    void getOrientation(int16_t*ori_x, int16_t* ori_y,int16_t* ori_z);
    void getAcceleration(int16_t *acc_x, int16_t* acc_y,int16_t* acc_z);
    void getRotationalSpeed(int16_t *rot_x, int16_t* rot_y,int16_t* rot_z);
    void configureZGyro(uint8_t mode);
    void getTemperature(double* temp);
};
