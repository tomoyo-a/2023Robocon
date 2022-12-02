#include "path.h"
#include "moveBase.h"
#include "robot.h"

//蓝场
uint8_t BLUE_TAKE_BALL_PATH_NUM = BLUE_TAKE_BALL_PATH_NUM_DEF;
uint8_t BLUE_TAKING_BALL_PATH_NUM = BLUE_TAKING_BALL_PATH_NUM_DEF;//取箭第二段
uint8_t BLUE_PUT_BALL_PATH_NUM = BLUE_PUT_BALL_PATH_NUM_DEF;
uint8_t BLUE_PUTTING_SECOND_BALL_PATH = BLUE_PUTTING_SECOND_BALL_PATH_NUM_DEF;
uint8_t BLUE_PUTTING_THIRD_BALL_PATH = BLUE_PUTTING_THIRD_BALL_PATH_NUM_DEF;
uint8_t BLUE_PUTTING_FOURTH_BALL_PATH = BLUE_PUTTING_FOURTH_BALL_PATH_NUM_DEF;
uint8_t BLUE_PUTTING_FIFTH_BALL_PATH = BLUE_PUTTING_FIFTH_BALL_PATH_NUM_DEF;
uint8_t BLUE_PUTTING_FIVE_BALL_PATH = BLUE_PUTTING_FIVE_BALL_PATH_NUM_DEF;

//红场
uint8_t RED_TAKE_BALL_PATH_NUM = RED_TAKE_BALL_PATH_NUM_DEF;
uint8_t RED_TAKING_BALL_PATH_NUM =RED_TAKING_BALL_PATH_NUM_DEF;
uint8_t RED_PUT_BALL_PATH_NUM = RED_PUT_BALL_PATH_NUM_DEF;
uint8_t RED_PUTTING_SECOND_BALL_PATH = RED_PUTTING_SECOND_BALL_PATH_NUM_DEF;
uint8_t RED_PUTTING_THIRD_BALL_PATH = RED_PUTTING_THIRD_BALL_PATH_NUM_DEF;
uint8_t RED_PUTTING_FOURTH_BALL_PATH = RED_PUTTING_FOURTH_BALL_PATH_NUM_DEF;
uint8_t RED_PUTTING_FIFTH_BALL_PATH = RED_PUTTING_FIFTH_BALL_PATH_NUM_DEF;

uint8_t TEST_PATH_NUM = TEST_PATH_NUM_DEF;
uint8_t TEST1_PATH_NUM = TEST1_PATH_NUM_DEF;
uint8_t TEST2_PATH_NUM = TEST2_PATH_NUM_DEF;
uint8_t TEST3_PATH_NUM = TEST3_PATH_NUM_DEF;
uint8_t TEST4_PATH_NUM = TEST4_PATH_NUM_DEF;

//去取球区
Pose_t BlueTakeBAllPath[BLUE_TAKE_BALL_PATH_NUM_DEF]=
{								
{	250.00f	,	250.00f	,	-90.f	,	200.f/FIRST_PATH_PERCENT	},
{	267.98f	,	539.58f	,	-90.f	,	MAX_TAR_VEL	},
{	321.66f	,	824.70f	,	-90.f	,	MAX_TAR_VEL	},
{	410.20f	,	1101.00f	,	-90.f	,	MAX_TAR_VEL	},
{	532.25f	,	1364.21f	,	-90.f	,	MAX_TAR_VEL	},
{	685.93f	,	1610.30f	,	-90.f	,	MAX_TAR_VEL	},
{	834.21f	,	1850.10f	,	-90.f	,	MAX_TAR_VEL	},
{	925.57f	,	2073.85f	,	-90.f	,	MAX_TAR_VEL	},
{	981.28f	,	2309.04f	,	-90.f	,	MAX_TAR_VEL	},
{	1000.00f	,	2550.00f	,	-90.f	,	MAX_TAR_VEL	},
{	975.14f	,	2737.25f	,	-90.f	,	MAX_TAR_VEL	},
{	902.29f	,	2911.53f	,	-90.f	,	MAX_TAR_VEL	},
{	786.49f	,	3060.76f	,	-90.f	,	MAX_TAR_VEL	},
{	645.16f	,	3200.26f	,	-90.f	,	MAX_TAR_VEL	},
{	503.84f	,	3339.76f	,	-90.f	,	MAX_TAR_VEL	},
{	362.51f	,	3479.25f	,	-90.f	,	MAX_TAR_VEL	},
{	246.71f	,	3628.48f	,	-90.f	,	MAX_TAR_VEL	},
{	173.86f	,	3802.76f	,	-90.f	,	600.f/FIRST_PATH_PERCENT	},
{	173.00f	,	3990.00f	,	-90.f	,	400.f/FIRST_PATH_PERCENT	},
{	173.00f	,	4190.00f	,	-90.f	,	300.f/FIRST_PATH_PERCENT	},
{	173.00f	,	4395.00f	,	-90.f	,	300.f/FIRST_PATH_PERCENT    },
};	


//横着取球阶段
Pose_t BlueTakingBallPath[BLUE_TAKING_BALL_PATH_NUM_DEF]=
{								
{	250.00f	,	4395.00f	,	-90.f	,	200.f/SECOND_PATH_PERCENT	},
{	365.38f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	480.77f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	596.15f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	711.54f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	826.92f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	942.31f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	1057.69f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	1173.08f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	1288.46f	,	4395.00f	,	-90.f	,	500.f/SECOND_PATH_PERCENT},
{	1403.85f	,	4395.00f	,	-90.f	,	500.f/SECOND_PATH_PERCENT},
{	1519.23f	,	4395.00f	,	-90.f	,	400.f/SECOND_PATH_PERCENT},
{	1634.62f	,	4395.00f	,	-90.f	,	200.f/SECOND_PATH_PERCENT},
{	1850.00f	,	4395.00f	,	-90.f	,	0.f	},
};

//取完球，去放球区
Pose_t BluePutBallPAth[BLUE_PUT_BALL_PATH_NUM_DEF]=
{								
{	1851.00f	,	4395.00f	,	-90.f	,	200.f/THIRD_PATH_PERCENT	},
{	1661.63f	,	4031.18f	,	-10.f	,	3500.f	},
{	1481.72f	,	3667.51f	,	60.f	,	3500.f	},
{	1311.40f	,	3299.26f	,	80.f	,	3500.f	},
{	1150.79f	,	2926.67f	,	90.f	,	3500.f	},
{	1000.00f	,	2550.00f	,	90.f	,	3500.f	},
{	891.32f	,	2223.94f	,	90.f	,	3500.f	},
{	813.03f	,	1889.27f	,	90.f	,	3500.f	},
{	765.79f	,	1548.84f	,	90.f	,	3500.f	},
{	750.00f	,	1205.50f	,	90.f	,	400.f/THIRD_PATH_PERCENT	},
{	750.00f	,	955.50f	,	90.f	,	200.f/THIRD_PATH_PERCENT	},
{	750.00f	,	818.f	,	90.f	,	0.f	},
};

//放完第一个球去放第二个球
Pose_t BluePuttingSecondBallPath[BLUE_PUTTING_SECOND_BALL_PATH_NUM_DEF]=
{								
{	750.00f	,	818.f	,	90.00f	,	200.f/PUT_SED_PATH_PERCENT	},
{	769.23f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	788.46f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	807.69f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	826.92f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	846.15f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	865.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	884.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	903.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	923.08f	,	818.f	,	90.00f	,	500.f/PUT_SED_PATH_PERCENT	},
{	942.31f	,	818.f	,	90.00f	,	500.f/PUT_SED_PATH_PERCENT	},
{	961.54f	,	818.f	,	90.00f	,	400.f/PUT_SED_PATH_PERCENT	},
{	980.77f	,	818.f	,	90.00f	,	200.f/PUT_SED_PATH_PERCENT	},
{	1007.00f	,	818.f	,	90.00f	,	0.f	},
};

//放完第二个球去放第三个球
Pose_t BluePuttingThirdBallPath[BLUE_PUTTING_THIRD_BALL_PATH_NUM_DEF]=
{								
{	1007.00f,	818.f	,	90.00f	,	200.f/PUT_THIRD_PATH_PERCENT},
{	1019.23f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1038.46f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1057.69f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1076.92f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1096.15f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1115.38f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1134.62f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1153.85f,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1173.08f,	818.f	,	90.00f	,	200.f/PUT_THIRD_PATH_PERCENT	},
{	1192.31f,	818.f	,	90.00f	,	150.f/PUT_THIRD_PATH_PERCENT	},
{	1211.54f,	818.f	,	90.00f	,	100.f/PUT_THIRD_PATH_PERCENT	},
{	1230.77f,	818.f	,	90.00f	,	50.f/PUT_THIRD_PATH_PERCENT	},
{	1257.00f,	818.f	,	90.00f	,	0.f	},
};

//放完第三个球去放第四个球
Pose_t BluePuttingFourthBallPath[BLUE_PUTTING_FOURTH_BALL_PATH_NUM_DEF]=
{								
{	1257.00f	,	818.f	,	90.00f	,	200.f/PUT_FOURTH_PATH_PERCENT	},
{	1269.23f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1288.46f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1307.69f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1326.92f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1346.15f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1365.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1384.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1403.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1423.08f	,	818.f	,	90.00f	,	200.f/PUT_FOURTH_PATH_PERCENT	},
{	1442.31f	,	818.f	,	90.00f	,	150.f/PUT_FOURTH_PATH_PERCENT	},
{	1461.54f	,	818.f	,	90.00f	,	100.f/PUT_FOURTH_PATH_PERCENT	},
{	1480.77f	,	818.f	,	90.00f	,	50.f/PUT_FOURTH_PATH_PERCENT	},
{	1507.00f	,	818.f	,	90.00f	,	0.f	},
};

//放完第四个球去放第五个球
Pose_t BluePuttingFifthBallPath[BLUE_PUTTING_FIFTH_BALL_PATH_NUM_DEF]=
{								
{	1507.00f	,	818.f	,	90.00f	,	200.f/PUT_FIFTH_PATH_PERCENT	},
{	1519.23f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1538.46f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1557.69f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1576.92f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1596.15f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1615.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1634.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1653.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	1673.08f	,	818.f	,	90.00f	,	200.f/PUT_FIFTH_PATH_PERCENT	},
{	1692.31f	,	818.f	,	90.00f	,	150.f/PUT_FIFTH_PATH_PERCENT	},
{	1711.54f	,	818.f	,	90.00f	,	100.f/PUT_FIFTH_PATH_PERCENT	},
{	1730.77f	,	818.f	,	90.00f	,	50.f/PUT_FIFTH_PATH_PERCENT	},
{	1750.00f	,	818.f	,	90.00f	,	0.f	},
};

//边走边放球
Pose_t BluePuttingFiveBallPath[BLUE_PUTTING_FIVE_BALL_PATH_NUM_DEF]=
{								
{	730.00f	,	760.f	,	90.00f	,	200.f/PUT_FIFTH_PATH_PERCENT	},
{	841.11f	,	760.f	,	90.00f	,	MAX_TAR_VEL	},
{	952.22f	,	760.f	,	90.00f	,	MAX_TAR_VEL	},
{	1063.33f	,	760.f	,	90.00f	,	MAX_TAR_VEL	},
{	1174.44f	,	760.f	,	90.00f	,	MAX_TAR_VEL	},
{	1285.56f	,	760.f	,	90.00f	,	200.f/PUT_FIFTH_PATH_PERCENT	},
{	1396.67f	,	760.f	,	90.00f	,	150.f/PUT_FIFTH_PATH_PERCENT	},
{	1507.78f	,	760.f	,	90.00f	,	100.f/PUT_FIFTH_PATH_PERCENT	},
{	1618.89f	,	760.f	,	90.00f	,	50.f/PUT_FIFTH_PATH_PERCENT	},
{	1730.00f	,	760.f	,	90.00f	,	0.f	},
};

Pose_t testPath[TEST_PATH_NUM_DEF]=
{	
{	-252.00f	,	250.00f	,	-90.00f	,	200.f/PUT_SED_PATH_PERCENT	},
{	-271.23f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-290.46f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-309.69f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-328.92f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-348.15f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-367.38f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-386.62f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-405.85f	,	250.00f	,	-90.00f	,	MAX_TAR_VEL	},
{	-425.08f	,	250.00f	,	-90.00f	,	500.f/PUT_SED_PATH_PERCENT	},
{	-444.31f	,	250.00f	,	-90.00f	,	500.f/PUT_SED_PATH_PERCENT	},
{	-463.54f	,	250.00f	,	-90.00f	,	400.f/PUT_SED_PATH_PERCENT	},
{	-482.77f	,	250.00f	,	-90.00f	,	200.f/PUT_SED_PATH_PERCENT	},
{	-502.00f	,	250.00f	,	-90.00f	,	0.f	},
};
/////////////////////////////////////////////////////////////////红场///////////////////////////////////////////////////////////////////////

//去取球区
Pose_t RedTakeBAllPath[RED_TAKE_BALL_PATH_NUM_DEF]=
{								
{	-252.00f	,	250.00f	,	-90.00f	,	200.f/FIRST_PATH_PERCENT	},
{	-252.16f	,	442.31f	,	-90.00f	,	MAX_TAR_VEL	},
{	-253.38f	,	634.54f	,	-90.00f	,	MAX_TAR_VEL	},
{	-278.31f	,	825.11f	,	-90.00f	,	MAX_TAR_VEL	},
{	-323.96f	,	1011.84f	,	-90.00f	,	MAX_TAR_VEL	},
{	-385.83f	,	1193.87f	,	-90.00f	,	MAX_TAR_VEL	},
{	-459.48f	,	1371.49f	,	-90.00f	,	MAX_TAR_VEL	},
{	-540.96f	,	1545.68f	,	-90.00f	,	MAX_TAR_VEL	},
{	-626.79f	,	1717.77f	,	-90.00f	,	MAX_TAR_VEL	},
{	-713.83f	,	1889.26f	,	-90.00f	,	MAX_TAR_VEL	},
{	-799.04f	,	2061.65f	,	-90.00f	,	MAX_TAR_VEL	},
{	-879.25f	,	2236.42f	,	-90.00f	,	MAX_TAR_VEL	},
{	-950.98f	,	2414.83f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1014.65f	,	2596.28f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1097.92f	,	2769.34f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1205.91f	,	2928.34f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1327.25f	,	3077.50f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1454.09f	,	3222.04f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1580.01f	,	3367.38f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1697.52f	,	3519.54f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1794.41f	,	3685.36f	,	-90.00f	,	MAX_TAR_VEL	},
{	-1848.33f	,	3869.13f	,	-90.00f	,	MAX_TAR_VEL},
{	-1851.00f	,	4061.30f	,	-90.00f	,	500.f/FIRST_PATH_PERCENT	},
{	-1851.00f	,	4300.61f	,	-90.00f	,	300.f/FIRST_PATH_PERCENT	},
{	-1851.00f	,	4395.00f	,	-90.00f	,	300.f/FIRST_PATH_PERCENT	},
};	

//横着取球阶段
Pose_t RedTakingBallPath[RED_TAKING_BALL_PATH_NUM_DEF]=
{								
{	-1850.51f	,	4395.00f	,	-90.f	,	200.f/SECOND_PATH_PERCENT	},
{	-1723.55f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-1596.59f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-1469.62f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-1342.66f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-1215.70f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-1088.74f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-961.77f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-834.81f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL	},
{	-707.85f	,	4395.00f	,	-90.f	,	MAX_TAR_VEL },
{	-580.89f	,	4395.00f	,	-90.f	,	450.f/SECOND_PATH_PERCENT },
{	-453.92f	,	4395.00f	,	-90.f	,	300.f/SECOND_PATH_PERCENT },
{	-326.96f	,	4395.00f	,	-90.f	,	150.f/SECOND_PATH_PERCENT },
{	-149.00f	,	4395.00f	,	-90.f	,	0.f	},
};

//取完球，去放球区
Pose_t RedPutBallPAth[RED_PUT_BALL_PATH_NUM_DEF]=
{								
{	-149	,	4390	,	90.f	,	200.f/THIRD_PATH_PERCENT	},
{	-272.192	,	4254.71	,	90.f	,	MAX_TAR_VEL	},
{	-388.516	,	4113.475	,	90.f	,	MAX_TAR_VEL	},
{	-497.158	,	3966.253	,	90.f	,	MAX_TAR_VEL	},
{	-597.105	,	3812.999	,	90.f	,	MAX_TAR_VEL	},
{	-687.113	,	3653.713	,	90.f	,	MAX_TAR_VEL	},
{	-765.693	,	3488.498	,	90.f	,	MAX_TAR_VEL	},
{	-831.11	,	3317.654	,	90.f	,	MAX_TAR_VEL	},
{	-881.759	,	3141.868	,	90.f	,	MAX_TAR_VEL	},
{	-920.533	,	2963.05	,	90.f	,	MAX_TAR_VEL	},
{	-953.935	,	2783.133	,	90.f	,	MAX_TAR_VEL	},
{	-988.515	,	2603.443	,	90.f	,	MAX_TAR_VEL	},
{	-1031.861	,	2425.094	,	90.f	,	MAX_TAR_VEL	},
{	-1093.132	,	2253.59	,	90.f	,	MAX_TAR_VEL	},
{	-1190.693	,	2099.423	,	90.f	,	MAX_TAR_VEL	},
{	-1319.954	,	1970.132	,	90.f	,	MAX_TAR_VEL	},
{	-1457.786	,	1849.769	,	90.f	,	MAX_TAR_VEL	},
{	-1584.844	,	1718.361	,	90.f	,	MAX_TAR_VEL	},
{	-1678.699	,	1561.909	,	90.f	,	MAX_TAR_VEL},
{	-1732.453	,	1387.364	,	90.f	,	400.f/THIRD_PATH_PERCENT	},
{	-1750.f	,	1205.5	,	90.f	,	200.f/THIRD_PATH_PERCENT	},
{	-1750.f	,	818.f	,	90.f	,	0.f	},
};


//放完第一个球去放第二个球
Pose_t RedPuttingSecondBallPath[RED_PUTTING_SECOND_BALL_PATH_NUM_DEF]=
{								
{	-1750.f	,	818.f	,	90.00f	,	150.f/PUT_FIFTH_PATH_PERCENT	},
{	-1730.77f	,	818.f	,	90.00f	,	400.f/PUT_FIFTH_PATH_PERCENT	},
{	-1711.54f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1692.31f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1673.08f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1653.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1634.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1615.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1596.15f	,	818.f	,	90.00f	,	250.f/PUT_FOURTH_PATH_PERCENT	},
{	-1576.92f	,	818.f	,	90.00f	,	200.f/PUT_FOURTH_PATH_PERCENT	},
{	-1557.69f	,	818.f	,	90.00f	,	150.f/PUT_FOURTH_PATH_PERCENT	},
{	-1538.46f	,	818.f	,	90.00f	,	100.f/PUT_FOURTH_PATH_PERCENT	},
{	-1519.23f	,	818.f	,	90.00f	,	50.f/PUT_FOURTH_PATH_PERCENT	},
{	-1500.00f	,	818.f	,	90.00f	,	0.f	},
};

//放完第二个球去放第三个球
Pose_t RedPuttingThirdBallPath[RED_PUTTING_THIRD_BALL_PATH_NUM_DEF]=
{								
{	-1500.00f	,	818.f	,	90.00f	,	150.f/PUT_FOURTH_PATH_PERCENT	},
{	-1480.77f	,	818.f	,	90.00f	,	400.f/PUT_FIFTH_PATH_PERCENT	},
{	-1461.54f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1442.31f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1423.08f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1403.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1384.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1365.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1346.15f	,	818.f	,	90.00f	,	250.f/PUT_FOURTH_PATH_PERCENT	},
{	-1326.92f	,	818.f	,	90.00f	,	200.f/PUT_FOURTH_PATH_PERCENT	},
{	-1307.69f	,	818.f	,	90.00f	,	150.f/PUT_FOURTH_PATH_PERCENT	},
{	-1288.46f	,	818.f	,	90.00f	,	100.f/PUT_FOURTH_PATH_PERCENT	},
{	-1269.23f	,	818.f	,	90.00f	,	50.f/PUT_FOURTH_PATH_PERCENT	},
{	-1250.00f	,	818.f	,	90.00f	,	0.f	},
};

//放完第三个球去放第四个球
Pose_t RedPuttingFourthBallPath[RED_PUTTING_FOURTH_BALL_PATH_NUM_DEF]=
{								
{	-1250.00f	,	818.f	,	90.00f	,	150.f/PUT_THIRD_PATH_PERCENT},	
{	-1230.77f	,	818.f	,	90.00f	,	400.f/PUT_FIFTH_PATH_PERCENT	},
{	-1211.54f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1192.31f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1173.08f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1153.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1134.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1115.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-1096.15f	,	818.f	,	90.00f	,	250.f/PUT_FOURTH_PATH_PERCENT	},
{	-1076.92f	,	818.f	,	90.00f	,	200.f/PUT_FOURTH_PATH_PERCENT	},
{	-1057.69f	,	818.f	,	90.00f	,	150.f/PUT_FOURTH_PATH_PERCENT	},
{	-1038.46f	,	818.f	,	90.00f	,	100.f/PUT_FOURTH_PATH_PERCENT	},
{	-1019.23f	,	818.f	,	90.00f	,	50.f/PUT_FOURTH_PATH_PERCENT	},
{	-1000.00f	,	818.f	,	90.00f	,	0.f	},
};

//放完第四个球去放第五个球
Pose_t RedPuttingFifthBallPath[RED_PUTTING_FIFTH_BALL_PATH_NUM_DEF]=
{								
{	-1000.00f	,	818.f	,	90.00f	,	150.f/PUT_SED_PATH_PERCENT	},
{	-980.77f	,	818.f	,	90.00f	,	400.f/PUT_FIFTH_PATH_PERCENT	},
{	-961.54f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-942.31f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-923.08f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-903.85f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-884.62f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-865.38f	,	818.f	,	90.00f	,	MAX_TAR_VEL	},
{	-846.15f	,	818.f	,	90.00f	,	250.f/PUT_FOURTH_PATH_PERCENT	},
{	-826.92f	,	818.f	,	90.00f	,	200.f/PUT_FOURTH_PATH_PERCENT	},
{	-807.69f	,	818.f	,	90.00f	,	150.f/PUT_FOURTH_PATH_PERCENT	},
{	-788.46f	,	818.f	,	90.00f	,	100.f/PUT_FOURTH_PATH_PERCENT	},
{	-769.23f	,	818.f	,	90.00f	,	50.f/PUT_FOURTH_PATH_PERCENT	},
{	-750.00f	,	818.f	,	90.00f	,	0.f	},
};
