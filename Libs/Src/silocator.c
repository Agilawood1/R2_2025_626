#include "silocator.h"


// 全局变量
static SickData sickdatas_list[SICK_NUMS];
static float learning_rate = 0.002f;
static float derivation_rate = 0.0001f;
Vec3 SiLo_normalize(Vec3 v);

/**
 * @name 核心距离计算函数
 * @details 计算在某一个点，朝向某一个角度theta时的距离
 * @return 距离值
 */
float SiLo_Get_Radius(float x, float y, float theta, float length, float width)
{
    theta = fmod(theta, 2*PI);
    
    float theta_0 = (length - x == 0) ? PI/2 : atanf(y / (length - x));
    float theta_1 = (y == 0) ? PI/2 : atanf(x / y) + PI/2;
    float theta_2 = (x == 0) ? PI : atanf((width - y)/x) + PI;
    float theta_3 = (width - y == 0) ? 3*PI/2 : atanf((length - x)/(width - y)) + 3*PI/2;

    if(theta < theta_0) return (length - x) / cosf(theta);
    if(theta < theta_1) return y / cosf(PI/2 - theta);
    if(theta < theta_2) return x / cosf(PI - theta);
    if(theta < theta_3) return (width - y) / cosf(3*PI/2 - theta);
    return (length - x) / cosf(theta);
}

float total_cost = 0;
float test_error = 0;
float theoryR;
// 代价函数
float SiLo_Cost_Func(float prd_x, float prd_y, float prd_fai, 
                float court_len, float court_wid) 
{
    // 定义总的 代价误差
    total_cost = 0;

    // 开始循环，遍历每一路西克
    for(int i=0; i < SICK_NUMS; i++)
    {
			test_error = 0;
        // 拒绝距离大于 6m 的数据
        if (sickdatas_list[i].distan > 6) continue;
        // 西克的距离不可能小于 0
        if(sickdatas_list[i].distan <= 0) continue;
        
        // 计算理论值
        theoryR = SiLo_Get_Radius(prd_x, prd_y, 
            sickdatas_list[i].theta + prd_fai, court_len, court_wid);
        // 与西克实际距离值计算误差
        test_error = fabsf(theoryR - sickdatas_list[i].distan);
        
        sickdatas_list[i].error = test_error;
        // 误差过大的激光不采用
        // if(error < 1.5f) 
        total_cost += test_error;
    }
    return total_cost;
}




/**
 * @name 梯度下降算法
 * @details 执行梯度下降算法
 */
void SiLo_Grad_Decent(Vec3 chassis_tf, Vec3 *silocator_tf)
{
    const float court_len = COURT_LENGTH, court_wid = COURT_WIDE;
    
    float cost_base = SiLo_Cost_Func(chassis_tf.x, chassis_tf.y, chassis_tf.z, court_len, court_wid);
    
    // X方向偏导
    float dx = (cost_base - SiLo_Cost_Func(chassis_tf.x+derivation_rate, chassis_tf.y, chassis_tf.z, 
             court_len, court_wid)) / derivation_rate;
    // Y方向偏导
    float dy = (cost_base - SiLo_Cost_Func(chassis_tf.x, chassis_tf.y+derivation_rate, chassis_tf.z, 
             court_len, court_wid)) / derivation_rate;
    // 角度偏导
    float df = (cost_base - SiLo_Cost_Func(chassis_tf.x, chassis_tf.y, chassis_tf.z+derivation_rate, 
             court_len, court_wid)) / derivation_rate;
    
    // 限幅处理
    // dx = fmaxf(fminf(dx, 1.0f), -1.0f);
    // dy = fmaxf(fminf(dy, 1.0f), -1.0f);
    // df = fmaxf(fminf(df, 1.0f), -1.0f);
	
    // 限幅改为归一化
    Vec3 delta_vec = {dx, dy, df / 4.0};
    delta_vec = SiLo_normalize(delta_vec);

    // 更新预测值（通过改变 修正向量）
    silocator_tf->x += delta_vec.x * learning_rate;
    silocator_tf->y += delta_vec.y * learning_rate;
    silocator_tf->z = fmod(silocator_tf->z + delta_vec.z * learning_rate, 2*PI);
}

// 传感器数据更新（需根据实际硬件实现）
//  这里的 theta 应该输入角度！！函数会自动把它转成弧度制
void SiLo_Update_SickData(int id, float distan, float theta)
{
    if(id >= SICK_NUMS) return;
    sickdatas_list[id].distan = distan + SICK_DISTAN_TO_CENTER_METER;       // 因为西克 不可能 装在车体中心，所以需要补偿
    sickdatas_list[id].theta = theta * PI / 180.0;      // 转成弧度制
    sickdatas_list[id].id = id;
}



// 主处理循环
void SiLo_Process(void)
{
    static int feedback_cnt = 0;
    const int feedback_intv = 3;
    
    // SiLo_Grad_Decent();
    
    if(++feedback_cnt >= feedback_intv){
        feedback_cnt = 0;
        // 这里添加位置反馈到其他模块的代码
    }
}


// 初始化函数
void SiLo_init(void)
{
    for (int i = 0; i < SICK_NUMS; i++)
    {
        sickdatas_list[i].id = i;
    }
    sickdatas_list[0].theta = 0;
    sickdatas_list[1].theta = 0;
    sickdatas_list[2].theta = 0;
    sickdatas_list[3].theta = 0;
    sickdatas_list[4].theta = 0;
    sickdatas_list[5].theta = 0;
}

Vec3 SiLo_normalize(Vec3 v) 
{
    // 计算向量的模
    double magnitude = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

    // 检查模是否为零，如果为零则返回零向量
    if (magnitude == 0) {
        Vec3 zero_vector = {0, 0, 0};
        return zero_vector;
    }

    // 归一化向量
    Vec3 normalized_vector;
    normalized_vector.x = v.x / magnitude;
    normalized_vector.y = v.y / magnitude;
    normalized_vector.z = v.z / magnitude;

    return normalized_vector;
}
