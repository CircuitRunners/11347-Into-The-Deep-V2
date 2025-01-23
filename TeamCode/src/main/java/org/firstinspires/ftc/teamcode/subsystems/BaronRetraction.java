package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.ArmConstants.armToHighBucket;
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

public class BaronRetraction {
    private Telemetry telemetry;

    public DcMotor retractOne, retractTwo;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public RunAction toHighBucket;
    public PIDController RetractionPID;
    public static int target;
    public static double p = RETkP, i = RETkI, d = RETkD;
    public static double f = RETkF;

    public BaronRetraction(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        retractOne = hardwareMap.get(DcMotor.class, RETRACTION_ONE);
        retractTwo = hardwareMap.get(DcMotor.class, RETRACTION_TWO);

        retractOne.setDirection(DcMotorSimple.Direction.FORWARD);
        retractTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        retractOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RetractionPID = new PIDController(p, i, d);

        toHighBucket = new RunAction(this::toHighBucket);
    }

    public void updatePIDF() {
        if (!manual) {
            RetractionPID = new PIDController(p, i, d);

            retractOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            retractTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = RetractionPID.calculate(getPos(), target);
            double ticks_in_degrees = ROT_ticks_in_degree;
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double power = pid + ff;

            power = Range.clip(power, -1.0, 1.0);

            retractOne.setPower(power);
            retractTwo.setPower(power);

            telemetry.addData("armPos: ", getPos());
            telemetry.addData("target: ", target);
        }
    }

    public void manual(double power) {
        manual = true;

        retractOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        retractTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hang) {
            power = -0.75;
        }

        retractOne.setPower(power);
        retractTwo.setPower(power);
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
        pos = retractTwo.getCurrentPosition();
        return retractTwo.getCurrentPosition();
    }


    /** OPMODE UTIL */
    public void init() {
        RetractionPID.setPID(p, i, d);
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

