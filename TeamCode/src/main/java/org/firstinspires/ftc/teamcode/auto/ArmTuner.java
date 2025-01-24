package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmRot;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;
import org.firstinspires.ftc.teamcode.support.Actions;
import org.firstinspires.ftc.teamcode.support.SleepCommand;

import java.util.List;

@Config
@Autonomous
public class ArmTuner extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private ArmRot rot;
    private activeIntake intake;
    private int pathState = 0;
    public static int drive = 0, HIGH = 0;

    private final Pose startPose = new Pose(8.8,113.5, Math.toRadians(0));
    private final Pose preloadBasked = new Pose(20, 124, Math.toRadians(315)); // 315

    private PathChain toBucket, rerun;

    private Telemetry telem;

    public void buildPaths() {
        toBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadBasked)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadBasked.getHeading())
                .build();

        rerun = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadBasked), new Point(startPose)))
                .setLinearHeadingInterpolation(preloadBasked.getHeading(), startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if (!follower.isBusy()) {
                    rot.setTarget(drive);
                    setPathState(0);
                }
                break;
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(toBucket, false);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    rot.setTarget(HIGH);
                    Actions.runBlocking(new SleepCommand(1));
                    Actions.runBlocking(intake.pivotScore);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(rerun, false);
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

        rot = new ArmRot(hardwareMap, telemetry);
        intake = new activeIntake(hardwareMap);

        drive = 1000;
        HIGH = 7000;

        intake.pivotMove();
//        rot.update();

        buildPaths();

        telem = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telem.addLine("ready");
        telem.update();
    }

    @Override
    public void loop() {
        rot.update();
        follower.update();
        autonomousPathUpdate();

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
