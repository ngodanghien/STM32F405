///*
// * mpu.h
// *
// *  Created on: Jul 12, 2020
// *      Author: dangh
// */
//
#ifndef __MPU6050_H_
#define __MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif


void IMU_Setup();
void Read_IMU();

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_AGV_INC_MPU6050_H_ */
