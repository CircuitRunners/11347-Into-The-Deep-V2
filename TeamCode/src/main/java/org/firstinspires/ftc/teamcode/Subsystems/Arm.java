package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.CurrnetUnit;

import org.firstinspires.ftc.teamcode.support.RunAction;
import org.firstinspires.ftc.teamcode.support.Constants;

public class Arm extends SubSystemBase {
    public enum ArmRotations {
        REST(5),
        MID(100);

        public int position;

        ArmRotations(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    public DcMotorEx rotationOne, rotationTwo, retractOne, retractTwo;

    private PIDController controller;
    public double p = kP, i = kI, d = kD;
    public double f = kF;

    public static int rotationTarget = targetRotation;

    private AnalogInput encoder;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private double VOLTAGE_WHEN_TUNED = 13.0;

    public RunAction testDown, testUp;

    public Arm(HardwareMap hw) {
        controller = new PIDController(p, i, d);

        rotationOne = hardwareMap.get(DcMotorEx.class, ROTATION_ONE);
        rotationTwo = hardwareMap.get(DcMotorEx.class, ROTATION_TWO);

        retractOne = hardwareMap.get(DcMotorEx.class, RETRACTION_ONE);
        retratctTwo = hardwareMap.get(DcMotorEx.class, RETRACTION_TWO);

        encoder = hardwareMap.get(AnalogInput.class, "rotationEnc");

        resetEncoders();

        rotationTarget = getCurrentRotation();

        testDown = new RunAction(this::testDown);
        testUp = new RunAction(this::testUp);
        
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_TUNED / voltageSensor.getVoltage();
    }

    public void update() {
        controller.setPID(p, i, d);
        double armPos = getCurrentRotation();
        double pid = controller.calculate(armPos, rotationTarget);
        double ff = Math.sin(Math.toRadians(rotationTarget)) * f;

        double power = pid + ff;

        setRotationPower(power);
    }

    public int getCurrentRotation() {
        return rotationOne.getCurrentPosition;
        // return (encoder.getVoltage() / 3.2 * 360) % 360;
    }

    public void setRotationTarget(double target) {
        this.rotationTarget = target;
    }

    public double getArmCurrent() {
        return rotationOne.getCurrent(CurrentUnit.AMPS);
    }

    public double getRotationTarget() {
        return rotationTarget;
    }

    public void manual(double n) {
        setRotationTarget(target + n * ARM_SPEED);
    }

    public void testDown() {
        setRotationTarget(ArmRotations.REST.position);
    }

    public void testUp() {
        setRotationTarget(ArmRotations.MID.position);
    }

    public void resetEncoders() {
        rotationOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractionOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractionTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotationOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractionOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractionTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRotationPosition() {
        setArmTarget(0);
        rotationOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        retractionOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractionTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getRotationVelocity() {
        return rotationOne.getVelocity();
    }

    public double getVoltageComp() {
        return voltageComp;
    }

    public void setRotationPower(double power) {
        rotationOne.setPower(power);
        rotationTwo.setPower(power);
    }

    public void setRetractionPower(double power) {
        retractionOne.setPower(power);
        retractionTwo.setPower(power);
    }

    public void brake() {
        setAllPower(0);
    }
}
