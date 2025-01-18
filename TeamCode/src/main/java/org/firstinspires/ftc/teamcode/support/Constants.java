package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Arm Constants
    public static double ROT_ticks_in_degree = 0.0417;
    public static double RET_ticks_in_degree = 0.0521;
    public static double ROTkP = -0.003, ROTkI = 0, ROTkD = -0.00011, ROTkF = -0.1;
    public static double RETkP = 0.0002, RETkI = 0, RETkD = 0, RETkF = -0.04;
    public static double armStart = 0.0;
    public static int targetRotation = 0;
    public static int targetRetraction = 0;
    public static double ARM_SPEED = 5;
    public static double RETRACT_SPEED = 5;
    
    public static final String ROTATION_ONE = "backArmRotation";
    public static final String ROTATION_TWO = "frontArmRotation";
    public static final String RETRACTION_ONE = "leftRetraction";
    public static final String RETRACTION_TWO = "rightRetraction";

    // Drivebase Constants
    public static double pinpointXOffset = -36.0;
    public static double pinpointYOffset = -80.835;
    
}
