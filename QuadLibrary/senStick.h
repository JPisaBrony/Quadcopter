//Register definitions for Spark Fun 9 DOF Sensor Stick
//Product number SEN-10724
//Written by Victor Grounds

#ifndef SENSTICK_H
#define SENSTICK_H

//HMC5883L 3-Axis Digital Compass
#define magAddress 0x1E
#define magWrite 0x3C
#define magRead 0x3D
#define magCRA 0x00
#define magCRB 0x01
#define magMode 0x02
#define magXoutH 0x03
#define magXoutL 0x04
#define magZoutH 0x05
#define magZoutL 0x06
#define magYoutH 0x07
#define magYoutL 0x08
#define magStatus 0x09
#define magIDA 0x0A
#define magIDB 0x0B
#define magIDC 0x0C

//ADXL345 3-Axis Digital Accelerometer
#define accelDefaultAddress 0x1D
#define accelAddress 0x53//This is the alternate address. The sensor stick uses this one.
#define accelWrite 0x3A
#define accelRead 0x3B
#define accelTHRESH_TAP 0x1D
#define accelOFSX 0x1E
#define accelOFSY 0x1F
#define accelOFSZ 0x20
#define accelDUR 0x21
#define accelLatent 0x22
#define accelWindow 0x23
#define accelTHRESH_ACT 0x24
#define accelTHRESH_INACT 0x25
#define accelTIME_INACT 0x26
#define accelACT_INACT_CTL 0x27
#define accelTHRESH_FF 0x28
#define accelTIME_FF 0x29
#define accelTAP_AXES 0x2A
#define accelACT_TAP_STATUS 0x2B
#define accelBW_RATE 0x2C
#define accelPOWER_CTL 0x2D
#define accelINT_ENABLE 0x2E
#define accelINT_MAP 0x2F
#define accelINT_SOURCE 0x30
#define accelDATA_FORMAT 0x31
#define accelXoutL 0x32
#define accelXoutH 0x33
#define accelYoutL 0x34
#define accelYoutH 0x35
#define accelZoutL 0x36
#define accelZoutH 0x37
#define accelFIFO_CTL 0x38
#define accelFIFO_STATUS 0x39

//ITG-3200 3-Axis Gyro
#define gyroAddress 0x68
#define gyroAltAddress 0x69
#define gyroWHO_AM_I 0x00
#define gyroSMPLRT_DIV 0x15
#define gyroDLPF_FS 0x16
#define gyroINT_CFG 0x17
#define gyroINT_STATUS 0x1A
#define gyroTEMP_OUT_H 0x1B
#define gyroTEMP_OUT_L 0x1C
#define gyroXoutH 0x1D
#define gyroXoutL 0x1E
#define gyroYoutH 0x1F
#define gyroYoutL 0x20
#define gyroZoutH 0x21
#define gyroZoutL 0x22
#define gyroPWR_MGM 0x3E

//BMP085 Barometric Pressure Sensor
#define bmpAddress 0x77
#define bmpControlRegister 0xF4
#define bmpReadRegisters 0xF6
#define bmpCalibrationCoefficients 0xAA
#define bmpTemperature 0x2E
#define bmpLowPowerPressure 0x34
#define bmpStandardPressure 0x74
#define bmpHighResPressure 0xB4
#define bmpUltraHighResPressure 0xF4

#endif
