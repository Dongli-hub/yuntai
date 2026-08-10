#ifndef USER_LIB_H
#define USER_LIB_H
#include <stdint.h>
#include <math.h>

#define PI_F 3.14159265358979323846f

float normalize_angle_rad(float ang);

typedef __packed struct
{
    float input;
    float out;
    float min_value;
    float max_value;
    float frame_period;
} ramp_function_source_t;

typedef __packed struct
{
    float input;
    float out;
    float num[1];
    float frame_period;
} first_order_filter_type_t;

extern float invSqrt(float num);
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
extern void abs_limit(float *num, float Limit);
extern float sign(float value);
extern float float_deadline(float Value, float minValue, float maxValue);
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
extern float float_constrain(float Value, float minValue, float maxValue);
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
extern float loop_float_constrain(float Input, float minValue, float maxValue);
extern float theta_format(float Ang);

#define rad_format(Ang) loop_float_constrain((Ang), -PI_F, PI_F)

#endif
