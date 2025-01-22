package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.automations.universalAutomations;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;
import org.firstinspires.ftc.teamcode.support.Actions;
import org.firstinspires.ftc.teamcode.support.SleepCommand;

import java.util.List;

@Autonomous
public class PreloadBucket extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private Arm arm;
    private activeIntake intake;
    private int pathState = 0;

    private final Pose startPose = new Pose(8.8,113.5, Math.toRadians(0));
    private final Pose preloadBasked = new Pose(20, 124, Math.toRadians(135)); // 315

    private PathChain toBucket;

    private Telemetry telem;

    public void buildPaths() {
        toBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadBasked)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadBasked.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.drive);
                    setPathState(0);
                }
                break;
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(toBucket, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.max);
                    Actions.runBlocking(new SleepCommand(2));
                    Actions.runBlocking(arm.fullExtend);
                    Actions.runBlocking(new SleepCommand(3));
                    Actions.runBlocking(intake.pivotScore);
                }
                break;
            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);

        arm = new Arm(hardwareMap);
        intake = new activeIntake(hardwareMap);

        intake.pivotMove();

        buildPaths();

        telem = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telem.addLine("ready");
        telem.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        arm.update();

        telem.addData("Arm Pos", arm.getCurrentRotation());
        telem.addData("Arm Target", arm.getRotationTarget());
        telem.addData("Path State", pathState);
        telem.addData("Position", follower.getPose().toString());
        telem.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(-1);
    }
}
