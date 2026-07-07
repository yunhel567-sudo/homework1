#ifndef AIS_SYNC_H
#define AIS_SYNC_H

// 定义 AIS 数据结构体 (根据输入字段定义)
typedef struct {
    long long mmsi;     // 船舶识别码
    float rot;          // 转向率
    float sog;          // 对地航速 (节)
    double lon;         // 经度
    double lat;         // 纬度
    float cog;          // 对地航向 (度)
    float hdg;          // 船艏向 (度)
    float rudder;       // 舵角
    float leftRpm;      // 左主机转速
    float rightRpm;     // 右主机转速
    long long timestamp;// 时间戳
} AisData;

// 定义输出的相对运动参数结构体
typedef struct {
    float ownSpeed_ms;    // 自身船速 (米/秒)
    float targetSpeed_ms; // 目标船速 (米/秒)
    float ownHeading;     // 自身对地航向 (0-360度)
    float targetHeading;  // 目标对地航向 (0-360度)
    double distance;      // 时间对齐后的两船距离 (米)
    float trueBearing;    // 时间对齐后，目标船在自身船的真方位 (0-360度)
} RelativeMotionParams;

/**
 * @brief 将目标船的时间对齐并计算相对运动参数 (使用局部等距圆柱投影)
 * @param ownShip    [in] 自身船数据 (作为时间基准和空间原点)
 * @param targetShip [in] 附近船原始数据
 * @param params     [out] 供后续碰撞模块使用的相对运动参数
 */
void processAndAlignData(const AisData* ownShip, const AisData* targetShip, RelativeMotionParams* params);

#endif // AIS_SYNC_H