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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmRet;
import org.firstinspires.ftc.teamcode.subsystems.ArmRot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import java.util.List;

@Config
@Autonomous
public class testformatthew extends OpMode {
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
    public static int mainRot = 1500, sampleRot = 1340;
    public static int fullRetract = 0;
    public static int retTarget = 0;

    // INTAKE
    public static int SCORE = 0, MOVE = 0;
    public static double intakePower = 0.0;

    private Telemetry telem;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case -1:
                rot.setTarget((1800));
                if (pathTimer.getElapsedTimeSeconds() > 5) {
                    rot.setTarget(mainRot);
                    claw.open();
                    diffy.subDiffy();
                    setPathState(0);
                }
                pathTimer.resetTimer();
                break;
            case 0:
                rot.setTarget(sampleRot);

                    if (pathTimer.getElapsedTimeSeconds() > 2){
                        claw.close();
                    }//rot.setTarget(mainRot);
                    //setPathState(1);
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
        follower.setMaxPower(1);

        rot = new ArmRot(hardwareMap, telemetry);
        ret = new ArmRet(hardwareMap, telemetry);
        diffy = new Diffy(hardwareMap);
        claw = new Claw(hardwareMap);



        // ROTATION
        //rot.setTarget(1800);
        // EXTENSION
        // = 1380;


        // INTAKE
        MOVE = 300; //330
        SCORE = 65; //75




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
