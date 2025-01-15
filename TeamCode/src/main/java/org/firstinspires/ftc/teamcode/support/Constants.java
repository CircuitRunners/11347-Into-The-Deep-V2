package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Arm Constants
    public static double kP = 0.0, kI = 0, kD = 0.0, kF = 0.0;
    public static double armStart = 0.0;
    public static double targetRotation = 0.0;
    public static double ARM_SPEED = 5;
    
    public static final String ROTATION_ONE = "backArmRotation";
    public static final String ROTATION_TWO = "frontArmRotation";
    public static final String RETRACTION_ONE = "leftRetraction";
    public static final String RETRACTION_TWO = "rightRetraction";

    // Drivebase Constants
    public static double pinpointXOffset = 0.0;
    public static double pinpointYOffset = 0.0;
    
}
