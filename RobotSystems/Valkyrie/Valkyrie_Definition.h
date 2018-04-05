#ifndef VALKYRIE_DEFINITION
#define VALKYRIE_DEFINITION


namespace valkyrie{
    // Simple version
    constexpr int num_q = 35;
    constexpr int num_qdot = 34;
    constexpr int num_act_joint = 28;

    constexpr int num_virtual = 6;
    constexpr double servo_rate = 0.001;
};

namespace valkyrie_link{
    constexpr int pelvis = 0;
    constexpr int torso = 1;
    constexpr int rightCOP_Frame = 2;
    constexpr int leftCOP_Frame = 3;
    constexpr int rightPalm = 4;
    constexpr int leftPalm = 5;
    constexpr int head = 6;
}

namespace valkyrie_joint{
    constexpr int  leftHipYaw = 0;
    constexpr int  leftHipRoll = 1;
    constexpr int  leftHipPitch = 2;
    constexpr int  leftKneePitch = 3;
    constexpr int  leftAnklePitch = 4;
    constexpr int  leftAnkleRoll = 5;

    constexpr int  rightHipYaw = 6;
    constexpr int  rightHipRoll = 7;
    constexpr int  rightHipPitch = 8;
    constexpr int  rightKneePitch = 9;
    constexpr int  rightAnklePitch = 10;
    constexpr int  rightAnkleRoll = 11;
        
    constexpr int  torsoYaw = 12;
    constexpr int  torsoPitch = 13;
    constexpr int  torsoRoll = 14;

    constexpr int  leftShoulderPitch = 15;
    constexpr int  leftShoulderRoll = 16;
    constexpr int  leftShoulderYaw = 17;
    constexpr int  leftElbowPitch = 18;
    constexpr int  leftForearmYaw = 19;

    constexpr int  lowerNeckPitch = 20;
    constexpr int  neckYaw = 21;
    constexpr int  upperNeckPitch = 22;

    constexpr int  rightShoulderPitch = 23;
    constexpr int  rightShoulderRoll = 24;
    constexpr int  rightShoulderYaw = 25;
    constexpr int  rightElbowPitch = 26;
    constexpr int  rightForearmYaw = 27;

}
#endif
