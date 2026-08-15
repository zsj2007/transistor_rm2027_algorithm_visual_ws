// Params.h
#ifndef PARAMS_H
#define PARAMS_H

// 枚举类型，表示敌方颜色
struct Params {
    enum EnemyColor {
        BLUE,   // 蓝色
        RED,    // 红色
        GREEN,   // 绿色 (作为默认)
        BOTH
    };

    // 默认的敌方颜色设置为蓝色
    EnemyColor enemy_color ;

    // 灯条检测参数
    int min_light_height;
    int light_min_area;
    int light_max_area;
    float max_light_wh_ratio;
    float min_light_wh_ratio;
    float light_max_tilt_angle;
};

#endif // PARAMS_H
