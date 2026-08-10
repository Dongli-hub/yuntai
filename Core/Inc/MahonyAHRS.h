#ifndef MahonyAHRS_h
#define MahonyAHRS_h

extern volatile float twoKp;
extern volatile float twoKi;

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#endif
