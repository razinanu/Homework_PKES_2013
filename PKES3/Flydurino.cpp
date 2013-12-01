#include "Flydurino.h"

Flydurino::Flydurino(){
  
    mpu6050Connected=0;
    hmc5883lConnected=0;
  
      // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
   
    mpu6050.initialize();
    if (mpu6050.testConnection()){
      mpu6050Connected=1;
    }
    
    // configuration of the compass module
    // activate the I2C bypass to directly access the Sub I2C 
    mpu6050.setI2CMasterModeEnabled(0);
    mpu6050.setI2CBypassEnabled(1);

    if (hmc5883l.testConnection()) {
        hmc5883lConnected=1;
        hmc5883l.initialize();
        
        // unfourtunally 
        // hmc5883l.setMode(HMC5883L_MODE_CONTINUOUS); 
        // does not work correctly. I used the following command to 
        // "manually" switch on continouse measurements
        I2Cdev::writeByte(HMC5883L_DEFAULT_ADDRESS,
                          HMC5883L_RA_MODE,
                          HMC5883L_MODE_CONTINUOUS);
        
        // the HMC5883l is configured now, we switch back to the MPU 6050
        mpu6050.setI2CBypassEnabled(0);
    
        // X axis word
        mpu6050.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80); 
        mpu6050.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
        setSlaveControl(0);
    
        // Y axis word
        mpu6050.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
        mpu6050.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
        setSlaveControl(1);
    
        // Z axis word
        mpu6050.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
        mpu6050.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
        setSlaveControl(2);
    
        mpu6050.setI2CMasterModeEnabled(1);
    }
    
    // activate temperature MPU 6050 sensor
    mpu6050.setTempSensorEnabled(true);
}

int8_t Flydurino::getmpu6050State(){
  return mpu6050Connected;
}

int8_t Flydurino::gethmc5883lState(){
  return hmc5883lConnected;
}

void Flydurino::setSlaveControl(uint8_t slaveID){
    mpu6050.setSlaveEnabled(slaveID, true);
    mpu6050.setSlaveWordByteSwap(slaveID, false);
    mpu6050.setSlaveWriteMode(slaveID, false);
    mpu6050.setSlaveWordGroupOffset(slaveID, false);
    mpu6050.setSlaveDataLength(slaveID, 2);
}

void Flydurino::getOrientation(int16_t* ori_x, int16_t* ori_y,int16_t* ori_z){
    *ori_x=mpu6050.getExternalSensorWord(0);
    *ori_y=mpu6050.getExternalSensorWord(2);
    *ori_z=mpu6050.getExternalSensorWord(4);
}

void Flydurino::getAcceleration(int16_t *acc_x, int16_t* acc_y,int16_t* acc_z){
   mpu6050.getAcceleration(acc_x, acc_y, acc_z);
}

void Flydurino::getRotationalSpeed(int16_t *rot_x, int16_t* rot_y,int16_t* rot_z){
   mpu6050.getRotation(rot_x, rot_y, rot_z);
}

void Flydurino::configureZGyro(uint8_t mode){
   mpu6050.setDLPFMode(mode);
   //mpu6050.setZGyroOffset(offset);
   //mpu6050.setZGyroOffsetTC(50);
}

void Flydurino::getTemperature(double* temp){
    *temp=((double) mpu6050.getTemperature()) /340.0 + 36.53;
}

