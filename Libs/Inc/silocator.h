#ifndef SILOCATOR
#define SILOCATOR

#include "std_msg.h"
#include <stdbool.h>
#include "math.h"


#define SICK_NUMS 8
#define PI 3.14159265358979323846f

#define SICK_DISTAN_TO_CENTER_METER 0.4
// #define SICK_DISTAN_TO_CENTER_METER 0

#define COURT_WIDE 7.94
#define COURT_LENGTH 9.77

void SiLo_init(void);
void SiLo_Process(void);
void SiLo_Update_SickData(int id, float distan, float theta);
void SiLo_Grad_Decent(Vec3 chassis_tf, Vec3 *silocator_tf);
float SiLo_Cost_Func(float prd_x, float prd_y, float prd_fai, 
    float court_len, float court_wid);
float SiLo_Get_Radius(float x, float y, float theta, float length, float width);

static SickData sickdatas_list[SICK_NUMS];
static float learning_rate;
static float derivation_rate;

#endif