package org.firstinspires.ftc.teamcode.configs;

import static org.firstinspires.ftc.teamcode.configs.FieldConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.BaronArm;
import org.firstinspires.ftc.teamcode.subsystems.BaronRetraction;

public class Auto {
    private RobotStart startLocation;
    public BaronArm arm;
    public BaronRetraction ret;

    public Follower follower;
    public Telemetry telemetry;

    public boolean actionBusy, rotPID = true, retPID = true;
    public double rotManual = 0, retManual = 0;
    public Timer bucketTimer = new Timer();
    public int actionState = -1, rotState = -1, retState = -1;

    public Path toBucket;
    public PathChain preloadPark;
    public Pose startPose, preloadBucket, parkPose;

    public Auto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
        arm = new BaronArm(hardwareMap, telemetry);
        ret = new BaronRetraction(hardwareMap, telemetry);

        Constants.setConstants(FConstants.class, LConstants.class);
        this.follower = follower;
        this.telemetry = telemetry;

        startLocation = isBlue ? (isBucket ? RobotStart.BLUE_BUCKET : RobotStart.BLUE_OBSERVATION) : (isBucket ? RobotStart.RED_BUCKET : RobotStart.RED_OBSERVATION);

        createPoses();
        buildPaths();

        init();
    }

    public void init() {
        arm.init();
        ret.init();
        telemetry.update();

        follower.setStartingPose(startPose);
    }

    public void start() {
        arm.start();
        ret.start();

        follower.setStartingPose(startPose);
    }

    public void update() {
        follower.update();

        if (!rotPID)
            arm.manual(rotManual);
        else
            arm.updatePIDF();

        if (!retPID)
            ret.manual(retManual);
        else
            ret.updatePIDF();

        score();

        telemetryUpdate();
    }

    public void createPoses() {
        switch (startLocation) {
            case BLUE_BUCKET:
                startPose = blueBucketStartPose;
                preloadBucket = blueBucketPreloadPose;
                parkPose = blueBucketParkPose;
                break;

            case RED_BUCKET:
                startPose = redBucketStartPose;
                preloadBucket = redBucketPreloadPose;
//                parkPose = redBucketParkPose;
                break;
        }

        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        if ((startLocation == RobotStart.BLUE_BUCKET) || (startLocation == RobotStart.RED_BUCKET)) {
            preloadPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(startPose), new Point(preloadBucket)))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preloadBucket.getHeading())
                    .build();


        }


    }

    public void score() {
        switch (actionState) {
            case 1:
                actionBusy = true;
                arm.toHighBucket();
//                ret.toHighBucket();
                setActionState(2);
                break;

            case 2:
                if (bucketTimer.getElapsedTimeSeconds() > 2) {
                    bucketTimer.resetTimer();
                    setActionState(3);
                }
                break;
        }
    }

    public void setActionState(int x) {
        actionState = x;
    }

    public void startScoring() {
        if (actionNotBusy()) {
            setActionState(1);
        }
    }

    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public boolean notBusy() {
        return ((!follower.isBusy() && actionNotBusy()));
    }

    public void telemetryUpdate() {
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Action Busy?: ", actionBusy);
        telemetry.addData("armPos: ", arm.getPos());
        telemetry.addData("rot Target: ", arm.getTarget());
        telemetry.addData("retPos: ", ret.getPos());
        telemetry.addData("ret Target: ", ret.getTarget());
//        telemetry.addData("arm State", arm.ArmState);
        telemetry.update();
    }
}
