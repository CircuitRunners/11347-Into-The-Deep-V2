package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Testing.ArmPID;
import org.firstinspires.ftc.teamcode.subsystems.Testing.TwoBitMechanisms;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;

@Disabled
@Autonomous
public class CommandAuto extends CommandOpMode {
    private DcMotor rot1, rot2, ret1, ret2;
    TwoBitMechanisms mechanisms;
    GoBildaPinpointDriver pinpoint;

    private Follower follower;
    private Timer pathTimer;
    private activeIntake intake;
    private int pathState = 0;

    final double HIGH = 6000;
    final double EXTEND = 57000;

    private final Pose startPose = new Pose(8.8,113.5, Math.toRadians(0));
    private final Pose preloadBasked = new Pose(20, 124, Math.toRadians(315)); // 315

    private PathChain toBucket;

    public void buildPaths() {
        toBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadBasked)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadBasked.getHeading())
                .build();
    }

    @Override
    public void initialize() {
        rot1 = hardwareMap.dcMotor.get("backArmRotation");
        rot2 = hardwareMap.dcMotor.get("frontArmRotation");
        ret1 = hardwareMap.dcMotor.get("leftRetraction");
        ret2 = hardwareMap.dcMotor.get("rightRetraction");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);

        mechanisms = new TwoBitMechanisms(rot1, rot2, ret1, ret2);

        buildPaths();

        schedule(new BulkCacheCommand(hardwareMap));

        telemetry.addLine("Init Done");
        telemetry.update();

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            follower.followPath(toBucket, true);
                        }),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new ArmPID(mechanisms, HIGH).withTimeout(1500)
                        )

                )
        ));
    }


//    switch (pathState) {
//        case 1:
//            if (!follower.isBusy()) {
//
//                setPathState(2);
//            }
//            break;
//        case 2:
//            if (!follower.isBusy()) {
//                follower.followPath(toBucket, true);
//                setPathState(3);
//            }
//            break;
//
//    }


}
