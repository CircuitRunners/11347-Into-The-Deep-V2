package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.support.Constants.LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.support.Constants.RIGHT_INTAKE;

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
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmRet;
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
    private ArmRet ret;
    private activeIntake intake;
    private CRServo leftIntake, rightIntake;
    private int pathState = 0;

    // ROTATION
    public static int drive = 0, HIGH = 0;
    public static int rotTarget = 0;

    //EXTENSION
    public static int fullExtend = 0;
    public static int fullRetract = 0;
    public static int retTarget = 0;

    // INTAKE
    public static int SCORE = 0, MOVE = 0;
    public static double intakePower = 0.0;

    private final Pose startPose = new Pose(8.8,113.5, Math.toRadians(0));
    private final Pose preloadBasked = new Pose(18, 126, Math.toRadians(315)); // 315
    private final Pose preloadSub = new Pose(34, 83, Math.toRadians(0));

    private PathChain toBucket, toSubPre, rerun;

    private Telemetry telem;

    public void buildPaths() {
        toBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadBasked)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadBasked.getHeading())
                .build();

        toSubPre = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadSub)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadSub.getHeading())
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
                    intake.setTarget(MOVE);
                    rot.setTarget(drive);
                    ret.setTarget(fullExtend);
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
                    setPathState(-2);
                }
                break;
            case -2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0) {

                    setPathState(2);
                }
                break;
            case 2:
                intake.setTarget(SCORE);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    intakePower = 0.7;
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.25) {
                    intakePower = 0.0;
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
        ret = new ArmRet(hardwareMap, telemetry);
        intake = new activeIntake(hardwareMap);

        leftIntake = hardwareMap.get(CRServo.class, LEFT_INTAKE);
        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE);

        // ROTATION
        drive = 1500;
        HIGH = 8000;

        // EXTENSION
        fullExtend = 57000;
        fullRetract = 0;

        // INTAKE
        MOVE = 300;
        SCORE = 75;
        intakePower = 0.0;

        intake.setTarget(MOVE);

        buildPaths();

        telem = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telem.addLine("ready");
        telem.update();
    }

    @Override
    public void loop() {
        rot.update();
        ret.update();
        follower.update();
        autonomousPathUpdate();

        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        telem.addData("Path State", pathState);
        telem.addData("Timer: ", pathTimer.getElapsedTimeSeconds());
        telem.addData("Position", follower.getPose().toString());
        telem.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(-1);
    }
}
