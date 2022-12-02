#ifndef _PATH_H
#define _PATH_H

#include "MotionCard.h"
#include "stdint.h"

#define PULSE_2_VEL(pulse) (((float)pulse/MOTOR_PULSE_PER_ROUND)/WHEEL_RATIO*PI*WHEEL_DIAMETER)
#define MAX_PLAN_VEL PULSE_2_VEL(MAX_MOTOR_SPEED)
#define MAX_TAR_VEL (10000.f)

//测试轨迹长度
//蓝场
#define BLUE_TAKE_BALL_PATH_NUM_DEF           (21)
#define BLUE_TAKING_BALL_PATH_NUM_DEF         (14)
#define BLUE_PUT_BALL_PATH_NUM_DEF            (12)
#define BLUE_PUTTING_SECOND_BALL_PATH_NUM_DEF (14)
#define BLUE_PUTTING_THIRD_BALL_PATH_NUM_DEF  (14)
#define BLUE_PUTTING_FOURTH_BALL_PATH_NUM_DEF (14)
#define BLUE_PUTTING_FIFTH_BALL_PATH_NUM_DEF  (14)
#define BLUE_PUTTING_FIVE_BALL_PATH_NUM_DEF   (10)//边走边放球
//红场
#define RED_TAKE_BALL_PATH_NUM_DEF            (25)
#define RED_TAKING_BALL_PATH_NUM_DEF          (14)
#define RED_PUT_BALL_PATH_NUM_DEF             (22)
#define RED_PUTTING_SECOND_BALL_PATH_NUM_DEF  (14)
#define RED_PUTTING_THIRD_BALL_PATH_NUM_DEF   (14)
#define RED_PUTTING_FOURTH_BALL_PATH_NUM_DEF  (14)
#define RED_PUTTING_FIFTH_BALL_PATH_NUM_DEF   (14)

#define TEST_PATH_NUM_DEF  (14)
#define TEST1_PATH_NUM_DEF (14)
#define TEST2_PATH_NUM_DEF (14)
#define TEST3_PATH_NUM_DEF (14)
#define TEST4_PATH_NUM_DEF (14)

//蓝场
extern uint8_t BLUE_TAKE_BALL_PATH_NUM;
extern uint8_t BLUE_TAKING_BALL_PATH_NUM;
extern uint8_t BLUE_PUT_BALL_PATH_NUM;
extern uint8_t BLUE_PUTTING_SECOND_BALL_PATH;
extern uint8_t BLUE_PUTTING_THIRD_BALL_PATH;
extern uint8_t BLUE_PUTTING_FOURTH_BALL_PATH;
extern uint8_t BLUE_PUTTING_FIFTH_BALL_PATH;
extern uint8_t BLUE_PUTTING_FIVE_BALL_PATH;
//红场
extern uint8_t RED_TAKE_BALL_PATH_NUM;
extern uint8_t RED_TAKING_BALL_PATH_NUM;
extern uint8_t RED_PUT_BALL_PATH_NUM;
extern uint8_t RED_PUTTING_SECOND_BALL_PATH;
extern uint8_t RED_PUTTING_THIRD_BALL_PATH;
extern uint8_t RED_PUTTING_FOURTH_BALL_PATH;
extern uint8_t RED_PUTTING_FIFTH_BALL_PATH;

extern uint8_t TEST_PATH_NUM;
extern uint8_t TEST1_PATH_NUM;
extern uint8_t TEST2_PATH_NUM;
extern uint8_t TEST3_PATH_NUM;
extern uint8_t TEST4_PATH_NUM;

//蓝场
extern Pose_t BlueTakeBAllPath[];
extern Pose_t BlueTakingBallPath[];
extern Pose_t BluePutBallPAth[];
extern Pose_t BluePuttingSecondBallPath[];
extern Pose_t BluePuttingThirdBallPath[];
extern Pose_t BluePuttingFourthBallPath[];
extern Pose_t BluePuttingFifthBallPath[];
extern Pose_t BluePuttingFiveBallPath[];
//红场
extern Pose_t RedTakeBAllPath[];
extern Pose_t RedTakingBallPath[];
extern Pose_t RedPutBallPAth[];
extern Pose_t RedPuttingSecondBallPath[];
extern Pose_t RedPuttingThirdBallPath[];
extern Pose_t RedPuttingFourthBallPath[];
extern Pose_t RedPuttingFifthBallPath[];

extern Pose_t testPath[];
extern Pose_t test1Path[];
extern Pose_t test2Path[];
extern Pose_t test3Path[];
extern Pose_t test4Path[];

#endif


