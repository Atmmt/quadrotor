#include "bsp_imu.h"
#include "zf_common_headfile.h"


#include <math.h>
#include <string.h>
#include <stdio.h>


static IMU_DATA_t icm_data_lpf;
IMU_DATA_t re_icm_data;

static uint8_t IMU963RA_Init(void)
{
    uint8_t init_v = imu963ra_init();
    
    imu963ra_get_acc();
    imu963ra_get_gyro();

    icm_data_lpf.accel.x = imu963ra_acc_x*0.002391410444f;
    icm_data_lpf.accel.y = imu963ra_acc_y*0.002391410444f;
    icm_data_lpf.accel.z = imu963ra_acc_z*0.002391410444f;
    // 低通滤波处理陀螺仪数据
    icm_data_lpf.gyro.x = imu963ra_gyro_x*0.06993006993f* 0.01745329252f;
    icm_data_lpf.gyro.y = imu963ra_gyro_y*0.06993006993f* 0.01745329252f;
    icm_data_lpf.gyro.z = imu963ra_gyro_z*0.06993006993f* 0.01745329252f;


    re_icm_data.accel.x = imu963ra_acc_x*0.002391410444f;
    re_icm_data.accel.y = imu963ra_acc_y*0.002391410444f;
    re_icm_data.accel.z = imu963ra_acc_z*0.002391410444f;
    // 低通滤波处理陀螺仪数据
    re_icm_data.gyro.x = imu963ra_gyro_x*0.06993006993f* 0.01745329252f;
    re_icm_data.gyro.y = imu963ra_gyro_y*0.06993006993f* 0.01745329252f;
    re_icm_data.gyro.z = imu963ra_gyro_z*0.06993006993f* 0.01745329252f;
    return init_v;
}



static IMU_DATA_t ZeroPoint = { 0 };//零点
static ATTITUDE_DATA_t ZeroAttitude = { 0 };

// 获取9轴无单位数据
static void ImuUpdate(IMU_DATA_t *data)
{
  
        float a=0.9;
        float acc[3];
        float gyro[3];
        
        imu963ra_get_acc();
        imu963ra_get_gyro();
//        imu963ra_get_mag();
        
        gyro[0] = imu963ra_gyro_x;
        gyro[1] = imu963ra_gyro_y;
        gyro[2] = imu963ra_gyro_z;
        
        acc[0]  = imu963ra_acc_x;
        acc[1]  = imu963ra_acc_y;
        acc[2]  = imu963ra_acc_z;
        
        gyro[0] *=  0.06993006993f; // 单位转换°/s
        gyro[1] *=  0.06993006993f;
        gyro[2] *=  0.06993006993f;
        
        
        gyro[0] *=  0.01745329252f; // 单位转换rad/s
        gyro[1] *=  0.01745329252f;
        gyro[2] *=  0.01745329252f;
        
        
        acc[0]  *=  0.002391410444f; // 单位转换  m/s-2
        acc[1]  *=  0.002391410444f;
        acc[2]  *=  0.002391410444f;
        
        
        icm_data_lpf.accel.x = icm_data_lpf.accel.x + ((0.005f / (7.9617834e-3 + 0.005f)) * ((float)acc[0] - icm_data_lpf.accel.x)); // 低通滤波
        icm_data_lpf.accel.y = icm_data_lpf.accel.y + ((0.005f / (7.9617834e-3 + 0.005f)) * ((float)acc[1] - icm_data_lpf.accel.y));
        icm_data_lpf.accel.z = icm_data_lpf.accel.z + ((0.005f / (7.9617834e-3 + 0.005f)) * ((float)acc[2] - icm_data_lpf.accel.z));
        // 低通滤波处理陀螺仪数据
        icm_data_lpf.gyro.x = icm_data_lpf.gyro.x + ((0.005f / (7.9617834e-3 + 0.005f)) * ((float)gyro[0] - icm_data_lpf.gyro.x));
        icm_data_lpf.gyro.y = icm_data_lpf.gyro.y + ((0.005f / (7.9617834e-3 + 0.005f)) * ((float)gyro[1] - icm_data_lpf.gyro.y));
        icm_data_lpf.gyro.z = icm_data_lpf.gyro.z + ((0.005f / (7.9617834e-3 + 0.005f)) * ((float)gyro[2] - icm_data_lpf.gyro.z));

        re_icm_data.gyro.x  = (float)re_icm_data.gyro.x*(1-a) +(float)icm_data_lpf.gyro.x*a - ZeroPoint.gyro.x;
        re_icm_data.gyro.y  = (float)re_icm_data.gyro.y*(1-a) +(float)icm_data_lpf.gyro.y*a - ZeroPoint.gyro.y;
        re_icm_data.gyro.z  = (float)re_icm_data.gyro.z*(1-a) +(float)icm_data_lpf.gyro.z*a - ZeroPoint.gyro.z;
        re_icm_data.accel.x = (float)re_icm_data.accel.x*(1-a)+(float)icm_data_lpf.accel.x*a - ZeroPoint.accel.x;
        re_icm_data.accel.y = (float)re_icm_data.accel.y*(1-a)+(float)icm_data_lpf.accel.y*a - ZeroPoint.accel.y;
        re_icm_data.accel.z = (float)re_icm_data.accel.z*(1-a)+(float)icm_data_lpf.accel.z*a - ZeroPoint.accel.z;

        
//        re_icm_data.gyro.x  = gyro[0];
//        re_icm_data.gyro.y  = gyro[1];
//        re_icm_data.gyro.z  = gyro[2];
//        re_icm_data.accel.x = acc[0];
//        re_icm_data.accel.y = acc[1];
//        re_icm_data.accel.z = acc[2];
        
        

        memcpy(data,&re_icm_data,sizeof(IMU_DATA_t));// 拷贝数据
}

//标定零点的方法
static void setZeroPoint_axis(const IMU_DATA_t* point) 
{
	memcpy(&ZeroPoint,point,sizeof(IMU_DATA_t));
}

static void setZeroPoint_attitude(const ATTITUDE_DATA_t* attitude)
{
	memcpy(&ZeroAttitude,attitude,sizeof(ATTITUDE_DATA_t));
}

static const float Kp = -0.6;   // 比例系数
static const float Ki = -0.0001; // 积分系数

//快速计算 1/sqrt(x)
static float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 

static void attitudeUpdate(IMU_DATA_t imudata, ATTITUDE_DATA_t *attitude) // 显示互补滤波（陀螺仪 + 加速度计）
{
    static float eInt[3] = {0}; // 初始化累计误差
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始化四元数
    float norm;
    float vx, vy, vz;
    float ex, ey, ez; // 误差变量
    float q0_t,q1_t,q2_t,q3_t;
    
    float halftime = 0.005f * 0.5f; // 2ms的采样周期
    
    
    // 辅助变量（简化变量名）
    float gx = imudata.gyro.x;  // 绕X轴旋转的角速度(rad/s)
    float gy = imudata.gyro.y;  // 绕Y轴旋转的角速度(rad/s)
    float gz = imudata.gyro.z;  // 绕Z轴旋转的角速度(rad/s)
    float ax = imudata.accel.x; // 沿X轴移动的加速度(m·s^-2)
    float ay = imudata.accel.y; // 沿Y轴移动的加速度(m·s^-2)
    float az = imudata.accel.z; // 沿Z轴移动的加速度(m·s^-2)

    // 辅助变量（避免冗长的计算表达式）
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
	
    // 提取等效旋转矩阵中的重力分量 
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = 1-2*(q1q1+q2q2);
	
    //加速度计归一化
    norm = Q_rsqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 计算误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 累计误差
    eInt[0] += ex;
    eInt[1] += ey;
    eInt[2] += ez;

    // PI控制
    gx += Kp * ex + Ki * eInt[0];
    gy += Kp * ey + Ki * eInt[1];
    gz += Kp * ez + Ki * eInt[2];

    // 更新四元数
    q0_t =  (-q1 * gx - q2 * gy - q3 * gz) * halftime;
    q1_t =  ( q0 * gx + q2 * gz - q3 * gy) * halftime;
    q2_t =  ( q0 * gy - q1 * gz + q3 * gx) * halftime;
    q3_t =  ( q0 * gz + q1 * gy - q2 * gx) * halftime;
	
    q0+=q0_t;
    q1+=q1_t;
    q2+=q2_t;
    q3+=q3_t;
	
    // 归一化（四元数）
    norm = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 四元数转换成欧拉角
    attitude -> roll  = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);
    attitude -> pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);
    attitude -> yaw   = atan2(2 *(q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

    // 零点漂移矫正
    attitude -> roll  -= ZeroAttitude.roll ;
    attitude -> pitch -= ZeroAttitude.pitch ;
    attitude -> yaw   -= ZeroAttitude.yaw;

}

//static uint8_t IMU963RA_ZeroPoint_Init(void)
//{
//  
//    uint8_t state = 0;
//
//    //持续稳定的次数
//    static uint8_t stablecount = 0;
//
//    static IMU_DATA_t LastImuData = { 0 };
//
//    //零点标定的次数
//    const uint8_t calibratetimes = 20;
//
//    IMU_DATA_t ZeroPoint_t = { 0 };
//
//    IMU_DATA_t  NowaxisVal = {0};
//    static IMU_DATA_t* zeropoint_offset_= {NULL};
//  
//     // 零点检测
//    while(zeropoint_offset_ == NULL)
//    {
//      state = 0;
//        ImuUpdate(&NowaxisVal);
//
//        if( fabs(NowaxisVal.gyro.x - LastImuData.gyro.x) < 0.01f ) state++;
//        if( fabs(NowaxisVal.gyro.y - LastImuData.gyro.y) < 0.01f ) state++;
//        if( fabs(NowaxisVal.gyro.z - LastImuData.gyro.z) < 0.01f ) state++;
//        if( fabs(NowaxisVal.accel.x- LastImuData.accel.x) < 0.03f ) state++;
//        if( fabs(NowaxisVal.accel.y - LastImuData.accel.y) < 0.03f ) state++;
//        if( fabs(NowaxisVal.accel.z - LastImuData.accel.z) < 0.03f ) state++;
//
//        if( state==6 )
//        {
//            stablecount++;
//            ZeroPoint_t.gyro.x +=NowaxisVal.gyro.x;
//            ZeroPoint_t.gyro.y +=NowaxisVal.gyro.y;
//            ZeroPoint_t.gyro.z +=NowaxisVal.gyro.z;
//            ZeroPoint_t.accel.x+=NowaxisVal.accel.x;
//            ZeroPoint_t.accel.y+=NowaxisVal.accel.y;
//            ZeroPoint_t.accel.z+=NowaxisVal.accel.z;
//
//            //持续的数据稳定,则认为当前数据稳定
//            if( stablecount == calibratetimes )
//            {
//                stablecount = 0;//完成6轴数据标定
//
//                //取连续稳定期间的数据平均值
//                ZeroPoint_t.gyro.x/=calibratetimes;
//                ZeroPoint_t.gyro.y/=calibratetimes;
//                ZeroPoint_t.gyro.z/=calibratetimes;
//                
//                ZeroPoint_t.accel.x/=calibratetimes;
//                ZeroPoint_t.accel.y/=calibratetimes;
//                ZeroPoint_t.accel.z/=calibratetimes;
//
////                ZeroPoint_t.accel.z-=9.8f;
//
//                zeropoint_offset_ = &ZeroPoint;
//                setZeroPoint_axis(&ZeroPoint_t); // 数据拷贝
//
//            }
//
//        }
//        else  //非连续的稳定数据,清除累计数值重新等待标定
//        {   
//                stablecount=0;
//                memset(&ZeroPoint,0,sizeof(IMU_DATA_t));
//        }
//        //保存上一次数据
//        memcpy(&LastImuData,&NowaxisVal,sizeof(IMU_DATA_t));
//    }
//}


//驱动挂载
IMUInterface_t UserIMU963RA = {
	.Init = IMU963RA_Init,
	.UpdateZeroPoint_axis = setZeroPoint_axis,
	.UpdateZeroPoint_attitude = setZeroPoint_attitude,
	.Update_9axisVal = ImuUpdate,
	.UpdateAttitude = attitudeUpdate
};


