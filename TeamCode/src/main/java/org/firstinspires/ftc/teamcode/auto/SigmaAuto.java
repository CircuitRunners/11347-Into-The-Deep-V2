package org.firstinspires.ftc.teamcode.auto;



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
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import java.util.List;

@Config
@Autonomous
public class SigmaAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private ArmRot rot;
    private ArmRet ret;
    private Diffy diffy;
    private Claw claw;
    private int pathState = -1;

    // ROTATION
    public static int drive = 0, HIGH = 0;
    public static int rotTarget = 0;

    //EXTENSION
    public static int fullExtend = 0, barExtend = 0, barScore = 0;
    public static int fullRetract = 0;
    public static int retTarget = 0;

    // INTAKE
    public static int SCORE = 0, MOVE = 0;
    public static double intakePower = 0.0;

    private final Pose startPose = new Pose(10,100, Math.toRadians(0));
    private final Pose SampleOne = new Pose(15, 120, Math.toRadians(90)); // 315
    private final Pose SampleTwo = new Pose(15, 128, Math.toRadians(-180));
    private final Pose SampleThree = new Pose(15, 130, Math.toRadians(180));

    private PathChain toBucket, toSampleOne, toSampleTwo;

    private Telemetry telem;

    public void buildPaths() {
        toBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(SampleOne)))
                .setLinearHeadingInterpolation(startPose.getHeading(), SampleOne.getHeading())
                .build();

        toSampleOne= follower.pathBuilder()
                .addPath(new BezierLine(new Point(SampleOne), new Point(SampleTwo)))
                .setLinearHeadingInterpolation(SampleOne.getHeading(), SampleTwo.getHeading())
//                .addPath(new BezierLine(new Point(preloadSub), new Point(sub)))
//                .setLinearHeadingInterpolation(preloadSub.getHeading(), sub.getHeading())
                .build();

        toSampleTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SampleTwo), new Point(SampleThree)))
                .setLinearHeadingInterpolation(SampleTwo.getHeading(), SampleThree.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if (!follower.isBusy()) {
                    follower.followPath(toBucket);
                    rot.setTarget(1380);
                    ret.setTarget(30000);
                    diffy.endDiffy();
                    if (ret.getCurrentRetraction() >28000) {
                        claw.open();
                    }
                    setPathState(0);
                }
                break;
            case 0:
                if (!follower.isBusy()) {
                    ret.setTarget(20);
                    if (ret.getCurrentRetraction() < 500) {
                        rot.setTarget(1500);
                        follower.followPath(toSampleOne, false);
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (!follower.isBusy()) {
//                    rot.setTarget(HIGH);
                    //setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    //ret.setTarget(barScore);
                    setPathState(3);
                }
                break;
            case 3:

                break;
            case 4:
                //
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
        diffy = new Diffy(hardwareMap);
        claw = new Claw(hardwareMap);



        // ROTATION
        drive = 4500;
        HIGH = 7700;

        // EXTENSION
        fullExtend = 57000;
        barExtend = 7500;
        barScore = 300;
        fullRetract = 0;

        // INTAKE
        MOVE = 300; //330
        SCORE = 65; //75
        intakePower = 0.0;



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
