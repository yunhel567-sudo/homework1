#include "ais_sync.h"
#include <math.h>

// 宏定义：角度转弧度，以及常数
#define PI 3.14159265358979323846
#define DEG2RAD(x) ((x) * PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / PI)

// 宏定义：单位换算与投影常数
#define KNOTS_TO_MS 0.5144444f        // 1节(knots) = 0.5144444 米/秒
#define R_EARTH 6378137.0             // WGS84 椭球体长半轴 (米)，近似地球半径

void processAndAlignData(const AisData* ownShip, const AisData* targetShip, RelativeMotionParams* params) {
    // 1. 基本参数赋值与换算 (节 -> 米/秒)
    params->ownSpeed_ms = ownShip->sog * KNOTS_TO_MS;
    params->targetSpeed_ms = targetShip->sog * KNOTS_TO_MS;
    params->ownHeading = ownShip->cog;
    params->targetHeading = targetShip->cog;
    
    // 2. 局部等距圆柱投影 (以自身船为原点，避免距离拉伸失真)
    double own_lat_rad = DEG2RAD(ownShip->lat);
    double own_lon_rad = DEG2RAD(ownShip->lon);
    double tar_lat_rad = DEG2RAD(targetShip->lat);
    double tar_lon_rad = DEG2RAD(targetShip->lon);
    
    // X轴(正东)：经度差乘以当前纬度的余弦值，进行球面距离补偿
    double initial_tar_x = R_EARTH * (tar_lon_rad - own_lon_rad) * cos(own_lat_rad);
    // Y轴(正北)：纬度差直接计算
    double initial_tar_y = R_EARTH * (tar_lat_rad - own_lat_rad);
    
    // 3. 计算时间差并插值
    long long dt = ownShip->timestamp - targetShip->timestamp;
    
    double aligned_tar_x = initial_tar_x;
    double aligned_tar_y = initial_tar_y;
    
    if (dt != 0) {
        // 计算目标船在X(东)和Y(北)方向的速度分量 (米/秒)
        // 航海坐标系以正北为0度，顺时针增加，因此x分量用sin，y分量用cos
        double tar_vx = params->targetSpeed_ms * sin(DEG2RAD(targetShip->cog));
        double tar_vy = params->targetSpeed_ms * cos(DEG2RAD(targetShip->cog));
        
        // 匀速直线插值，对齐到自身船时间点
        aligned_tar_x += tar_vx * dt;
        aligned_tar_y += tar_vy * dt;
    }
    
    // 4. 计算对齐后的两船距离 (单位：米)
    params->distance = sqrt(aligned_tar_x * aligned_tar_x + aligned_tar_y * aligned_tar_y);
    
    // 5. 计算目标船的真方位 (True Bearing)
    // 使用 atan2(x, y) 算出的是以 Y 轴(正北)为起点的弧度角
    double bearing_rad = atan2(aligned_tar_x, aligned_tar_y);
    float bearing_deg = RAD2DEG(bearing_rad);
    
    // 将方位角规范化到 0-360 度范围
    if (bearing_deg < 0) {
        bearing_deg += 360.0f;
    }
    params->trueBearing = bearing_deg;
}