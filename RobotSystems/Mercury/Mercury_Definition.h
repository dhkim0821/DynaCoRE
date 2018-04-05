#ifndef MERCURY_DEFINITION
#define MERCURY_DEFINITION

namespace mercury{
    constexpr int num_q = 13;
    constexpr int num_qdot = 12;
    constexpr int num_act_joint = 6;
    constexpr int num_virtual = 6;
    constexpr double servo_rate = 1.0/1500.0;
};

namespace mercury_link{
    constexpr int body = 0;
    constexpr int rightFoot = 1;
    constexpr int leftFoot = 2;
    constexpr int imu = 3;

    constexpr int LED_BODY_0 = 20;

    constexpr int LED_RLEG_0 = 21;
    constexpr int LED_RLEG_1 = 22;
    constexpr int LED_RLEG_2 = 23;
    constexpr int LED_RLEG_3 = 24;

    constexpr int LED_LLEG_0 = 25;
    constexpr int LED_LLEG_1 = 26;
    constexpr int LED_LLEG_2 = 27;
    constexpr int LED_LLEG_3 = 28;

    constexpr int LED_BODY_1 = 29;
    constexpr int LED_BODY_2 = 30;
};
#endif

