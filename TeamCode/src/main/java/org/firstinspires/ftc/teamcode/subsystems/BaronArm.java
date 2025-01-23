package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.ArmConstants.*;
import static org.firstinspires.ftc.teamcode.support.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.support.RunAction;

public class BaronArm {
//    public enum ArmState {
//        HIGH_BUCKET, INIT, SPECIMEN_GRAB, SPECIMEN_SCORE
//    }

//    public ArmState state;
    private Telemetry telemetry;

    public DcMotor rotationOne, rotationTwo;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public RunAction toHighBucket;
    public PIDController RotationPID;
    public static int target;
    public static double p = ROTkP, i = ROTkI, d = ROTkD;
    public static double f = ROTkF;


    public BaronArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        this.state = state;

        // Initialize motors
        rotationOne = hardwareMap.get(DcMotor.class, ROTATION_ONE);
        rotationTwo = hardwareMap.get(DcMotor.class, ROTATION_TWO);

        rotationOne.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        rotationOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        RotationPID = new PIDController(p, i, d);

        toHighBucket = new RunAction(this::toHighBucket);
    }

    public void updatePIDF() {
        if (!manual) {
            RotationPID = new PIDController(p, i, d);

            rotationOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rotationTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = RotationPID.calculate(getPos(), target);
            double ticks_in_degrees = ROT_ticks_in_degree;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            power = Range.clip(power, -1.0, 1.0);

            rotationOne.setPower(power);
            rotationTwo.setPower(power);

            telemetry.addData("armPos: ", getPos());
            telemetry.addData("target: ", target);
        }
    }

    public void manual(double power) {
        manual = true;

        rotationOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hang) {
            power = -0.75;
        }

        rotationOne.setPower(power);
        rotationTwo.setPower(power);
    }


    /** UTIL */
    public void targetCurrent() {
        setTarget(getPos());
        manual = false;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(int t) {
        target = t;
    }

    public void addToTarget(int t) {
        target += t;
    }

    public int getPos() {
        pos = rotationTwo.getCurrentPosition();
        return rotationTwo.getCurrentPosition();
    }


    /** OPMODE UTIL */
    public void init() {
        RotationPID.setPID(p, i, d);
        bottom = getPos();
    }

    public void start() {
        target = 0;
    }

    public void toHighBucket() {
        manual = false;
        setTarget(armToHighBucket);
    }
}
