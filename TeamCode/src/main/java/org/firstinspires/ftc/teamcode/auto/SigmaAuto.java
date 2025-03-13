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
    private final Pose bucket = new Pose(18.5, 120, Math.toRadians(-45)); // 315
    private final Pose bucket2 = new Pose(19, 121, Math.toRadians(-45));
    private final Pose SampleOne = new Pose(28, 117, Math.toRadians(0)); // 315
    private final Pose SampleTwo = new Pose(28,126, Math.toRadians(0));
    private final Pose SampleThree = new Pose(30, 125, Math.toRadians(-35));

    private PathChain toBucket, toSampleOne, toSampleTwo, toBucket1,toBucket2;

    private Telemetry telem;

    public void buildPaths() {
        toBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(bucket)))
                .setLinearHeadingInterpolation(startPose.getHeading(), bucket.getHeading())
                .build();

        toSampleOne= follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucket), new Point(SampleOne)))
                .setLinearHeadingInterpolation(bucket.getHeading(), SampleOne.getHeading())
//                .addPath(new BezierLine(new Point(preloadSub), new Point(sub)))
//                .setLinearHeadingInterpolation(preloadSub.getHeading(), sub.getHeading())
                .build();
        //toBucket1 =

        toSampleTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucket), new Point(SampleTwo)))
                .setLinearHeadingInterpolation(bucket.getHeading(), SampleTwo.getHeading())
                .build();
        toBucket1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SampleOne), new Point(bucket)))
                .setLinearHeadingInterpolation(SampleOne.getHeading(), bucket.getHeading())
                .build();
        toBucket2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SampleTwo), new Point(bucket2)))
                .setLinearHeadingInterpolation(SampleTwo.getHeading(), bucket2.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                if (!follower.isBusy()) {
                    follower.followPath(toBucket); // TODO: TRY MOVING FIRST CASE
                    //pathTimer.resetTimer();
                    setPathState(0);

                }
                pathTimer.resetTimer();
                break;
            case 0:
                if (!follower.isBusy()) {
                    if (rotTarget <1000) {
                        rot.setTarget(8300); // TODO: CHECK IF USING VARS FIXES
                        diffy.centerDiffy();
                    }
                    if (rot.getCurrentRotation() > 8000)
                    {

                        ret.setTarget(52000);
                        //claw.open();



                    }

                    if (ret.getCurrentRetraction() > 10000) {
                        if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                            pathTimer.resetTimer();
                            setPathState(1);

                        }

                    }

                }

                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (ret.getTarget() > 10000) {
                        diffy.endDiffy();
                        claw.open();
                        if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                            ret.setTarget(0);
                            diffy.centerDiffy();
                        }
                    }
                    if (rot.getTarget() > 5000) {
                        if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                            rot.setTarget(1500);
                            pathTimer.resetTimer();
                            setPathState(2);
                        }
                    }
                }
                //pathTimer.resetTimer();
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (ret.getTarget() < 500) {
//                        rotTarget = 1650; //extension = 46000
//                        retTarget = 30000;
//    //
    //                rotTarget = 1100;
    //                //retTarget = 57000;
    //                ret.setTarget(57000);
    //                fish = true;
                        diffy.subDiffy();
                        claw.open();


                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    rot.setTarget(1650);
                    ret.setTarget(30000);
                    setPathState(3);
                }

                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(toSampleOne);
                    setPathState(4);
                }
                pathTimer.resetTimer();

                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    if (rot.getTarget() > 1300) {
                        rot.setTarget(1300);

                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.7) {
                    claw.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.9) {
                    if (ret.getTarget() > 1400) {
                        ret.setTarget(0);
                        rot.setTarget(1500);
                        diffy.centerDiffy();
                        setPathState(5);
                    }
                }


                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(toBucket1); // TODO: TRY MOVING FIRST CASE
                    //pathTimer.resetTimer();
                    setPathState(6);

                }
                pathTimer.resetTimer();
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (rotTarget <1000) {
                        rot.setTarget(8300); // TODO: CHECK IF USING VARS FIXES
                        diffy.centerDiffy();
                    }
                    if (rot.getCurrentRotation() > 8000)
                    {

                        ret.setTarget(52000);
                        //claw.open();



                    }

                    if (ret.getCurrentRetraction() > 10000) {
                        if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                            pathTimer.resetTimer();
                            setPathState(7);

                        }

                    }

                }

                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (ret.getTarget() > 10000) {
                        diffy.endDiffy();
                        claw.open();
                        if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                            ret.setTarget(0);
                            diffy.centerDiffy();
                        }
                    }
                    if (rot.getTarget() > 5000) {
                        if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                            rot.setTarget(1500);
                            pathTimer.resetTimer();
                            setPathState(8);
                        }
                    }
                }
                //pathTimer.resetTimer();
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (ret.getTarget() < 500) {
//                        rotTarget = 1650; //extension = 46000
//                        retTarget = 30000;
//    //
                        //                rotTarget = 1100;
                        //                //retTarget = 57000;
                        //                ret.setTarget(57000);
                        //                fish = true;
                        diffy.subDiffy();
                        claw.open();


                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    rot.setTarget(1650);
                    ret.setTarget(30000);
                    setPathState(9);
                }

                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(toSampleTwo);
                    setPathState(10);
                }
                pathTimer.resetTimer();
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    if (rot.getTarget() > 1300) {
                        rot.setTarget(1300);

                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.7) {
                    claw.close();
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.9) {
                    if (ret.getTarget() > 1400) {
                        ret.setTarget(0);
                        rot.setTarget(1500);
                        diffy.centerDiffy();
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(toBucket2); // TODO: TRY MOVING FIRST CASE
                    //pathTimer.resetTimer();
                    setPathState(12);

                }
                pathTimer.resetTimer();
                break;
            case 12:
                if (!follower.isBusy()) {
                    if (rotTarget <1000) {
                        rot.setTarget(8300); // TODO: CHECK IF USING VARS FIXES
                        diffy.centerDiffy();
                    }
                    if (rot.getCurrentRotation() > 8000)
                    {

                        ret.setTarget(52000);
                        //claw.open();



                    }

                    if (ret.getCurrentRetraction() > 10000) {
                        if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                            pathTimer.resetTimer();
                            setPathState(13);

                        }

                    }

                }

                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (ret.getTarget() > 10000) {
                        diffy.endDiffy();
                        claw.open();
                        if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                            ret.setTarget(0);
                            diffy.centerDiffy();
                        }
                    }
                    if (rot.getTarget() > 5000) {
                        if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                            rot.setTarget(1500);
                            pathTimer.resetTimer();
                            //setPathState(14);
                        }
                    }
                }
                //pathTimer.resetTimer();
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


        rotTarget = 0;
        retTarget = 0;
        buildPaths();

        telem = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telem.addLine("ready");
        telem.update();
    }

    @Override
    public void loop() {
        rot.update();
        ret.update();
//        rot.setTarget(rotTarget);
//        ret.setTarget(retTarget);
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
