package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Arm Constants
    public static double ROT_ticks_in_degree = 0.0417;
    public static double RET_ticks_in_degree = 0.0521;
    public static double ROTkP = -0.0015, ROTkI = 0, ROTkD = -0.00011, ROTkF = -0.08;
    public static double RETkP = 0.0002, RETkI = 0, RETkD = 0, RETkF = -0.04;
    public static double armStart = 0.0; // DEPRECIATED
    public static int targetRotation = 0; // DEPRECIATED
    public static int targetRetraction = 0; // DEPRECIATED
    public static double ARM_SPEED = 5; // DEPRECIATED
    public static double RETRACT_SPEED = 5; // DEPRECIATED
    
    public static final String ROTATION_ONE = "backArmRotation";
    public static final String ROTATION_TWO = "frontArmRotation";
    public static final String RETRACTION_ONE = "leftRetraction";
    public static final String RETRACTION_TWO = "rightRetraction";

    // Drivebase Constants
    public static double pinpointXOffset = 1.41732; // MM = -36.0
    public static double pinpointYOffset = -3.18208661; // MM = -80.835 // 3.18208661

    // Active Intake Constants
    public static final String LEFT_INTAKE = "leftIntake";
    public static final String RIGHT_INTAKE = "rightIntake";
    public static final String PIVOT = "pivot";
    public static final String HOLD = "hold";
}
