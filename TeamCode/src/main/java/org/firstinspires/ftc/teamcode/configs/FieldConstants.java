package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class FieldConstants {
    public enum RobotStart {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public static final Pose blueBucketStartPose = new Pose(7.5, 78.75, Math.toRadians(180));
    public static final Pose redBucketStartPose = new Pose(8.8, 113.5, Math.toRadians(180));

    // Preload Poses
    public static final Pose blueBucketPreloadPose = new Pose(29.25, 78.375, Math.toRadians(180));
    public static final Pose redBucketPreloadPose = new Pose(20, 124, Math.toRadians(135));

    // Park Poses
    public static final Pose blueBucketParkPose = new Pose(62, 97.75, Math.toRadians(90));
    public static final Pose blueBucketParkControlPose = new Pose(60.25, 123.5);
}
