cmake_minimum_required(VERSION 3.10)

# 定义项目名称和使用的语言
project(ShipCollisionAvoidance C)

# 设置 C 语言标准 (推荐使用 C99 或 C11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)

# 收集所有的源文件
set(SOURCES
    main.c
    ais_sync.c
    collision_risk.c
    colregs_decision.c
    path_decision.c
)

# 生成可执行文件
add_executable(avoidance_system ${SOURCES})

# 链接数学库 (非常重要：因为代码中使用了 sin, cos, sqrt, atan2 等 math.h 中的函数)
# 在 Linux/macOS 下相当于编译时的 -lm 参数
target_link_libraries(avoidance_system m)