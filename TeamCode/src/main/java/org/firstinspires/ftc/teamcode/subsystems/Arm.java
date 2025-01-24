package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.support.RunAction;
import static org.firstinspires.ftc.teamcode.support.Constants.*;

@Config
public class Arm {

    public enum ArmRotations {
        DRIVE(300),
        GRAB(150),
        REST(500),
        MID(2000),
        MAX(8000);

        public int position;

        ArmRotations(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    public enum ArmRetractions {
        REST(-300),
        MID(-100),
        MAX(-57000);

        public int position;

        ArmRetractions(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    public DcMotorEx rotationOne, rotationTwo, retractOne, retractTwo;
    private PIDController RotationController, RetractionController;

    public double ROTp = ROTkP, ROTi = ROTkI, ROTd = ROTkD;
    public double RETp = RETkP, RETi = RETkI, RETd = RETkD;
    public double ROTf = ROTkF;
    public double RETf = RETkF;

    public int rotationTarget = 0;
    public int retractionTarget = 0;
    public boolean rotReached;
    public boolean retReached;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private static final double VOLTAGE_WHEN_TUNED = 13.0;

    public RunAction max, drive, grab, fullRet, fullExtend;

    public Arm(HardwareMap hardwareMap) {
        RotationController = new PIDController(ROTp, ROTi, ROTd);
        RetractionController = new PIDController(RETp, RETi, RETd);

        // Initialize motors
        rotationOne = hardwareMap.get(DcMotorEx.class, ROTATION_ONE);
        rotationTwo = hardwareMap.get(DcMotorEx.class, ROTATION_TWO);
        retractOne = hardwareMap.get(DcMotorEx.class, RETRACTION_ONE);
        retractTwo = hardwareMap.get(DcMotorEx.class, RETRACTION_TWO);

        retractTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        rotationOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        resetEncoders();
        resetTargets();

        rotationTarget = getCurrentRotation();
        retractionTarget = getCurrentRetraction();

        // Initialize actions
        max = new RunAction(this::max);
        drive = new RunAction(this::drive);
        grab = new RunAction(this::grab);

        fullRet = new RunAction(this::fullRet);
        fullExtend = new RunAction(this::fullExtend);

        // Voltage compensation
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltageComp = VOLTAGE_WHEN_TUNED / voltageSensor.getVoltage();
    }

    public void update() {
        RetractionController.setPID(RETp, RETi, RETd);
        RotationController.setPID(ROTp, ROTi, ROTd);

        double rotPos = getCurrentRotation();
        double retPos = getCurrentRetraction();

        double rotPID = RotationController.calculate(rotPos, rotationTarget);
        double retPID = RetractionController.calculate(retPos, retractionTarget);

        double ROTff = Math.sin(Math.toRadians(rotationTarget / ROT_ticks_in_degree)) * ROTf;
        double RETff = Math.sin(Math.toRadians(retractionTarget / RET_ticks_in_degree)) * RETf;

        // CLIPS RANGE OF MOTORS TO BE BETWEEN -1 AND 1
        double ROTpower = Range.clip(rotPID + ROTff, -1.0, 1.0);
        double RETpower = Range.clip(retPID + RETff, -1.0, 1.0);

        // SETS MOTORS TO CLIPPED POWER
        setRotationPower(ROTpower * 0.8);
//        setRetractionPower(RETpower);
    }   


    /** GETTER METHODS FOR POSITIONS */
    public int getCurrentRotation() {
        return rotationTwo.getCurrentPosition();
    }
    public int getCurrentRetraction() {
        return retractTwo.getCurrentPosition();
    }
    public int getRotationTarget() {
        return rotationTarget;
    }
    public int getRetractionTarget() {
        return retractionTarget;
    }


    /** TARGET SETTER METHODS */
//    public void setRotationTarget(int target) {
//        this.rotationTarget = target;
//    }
//    public void setRetractionTarget(int target) {
//        this.retractionTarget = target;
//    }
    /** TARGET SETTER METHODS */
    public void setRotationTarget(int target) {
        this.rotationTarget = Range.clip(target, 0, ArmTest.ArmRotations.MAX.position);
        RotationController.setSetPoint(target);
    }
    public void setRetractionTarget(int target) {
        this.retractionTarget = Range.clip(target, 0, ArmTest.ArmRetractions.MAX.position);
        RetractionController.setSetPoint(target);
    }

    /** MANUAL SETTERS */
    public void ROTManual(double pos) {
        setRotationTarget(getCurrentRotation() + (int) (pos * 5));
    }
    public void RETManaul(int d) {
        setRetractionTarget(getCurrentRetraction() + (d * 50));
    }


    /** RESET ALL ENCODERS */
    public void resetEncoders() {
        rotationOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotationOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetTargets() {
        setRotationTarget(0);
        setRetractionTarget(0);
    }


    /** SET POWER METHODS */
    public void setRotationPower(double power) {
        rotationOne.setPower(power);
        rotationTwo.setPower(power);
    }
    public void setRetractionPower(double power) {
        retractOne.setPower(power);
        retractTwo.setPower(power);
    }
    public void setAllPower(double power) {
        setRotationPower(power);
        setRetractionPower(power);
    }


    /** BRAKE */
    public void brake() {
        setAllPower(0);
    }


    /** ROTATION COMMANDS */
    public void max() {
        setRotationTarget(ArmRotations.MAX.position);
    }
    public void drive() {
        setRotationTarget(ArmRotations.DRIVE.position);
    }
    public void grab() {
        setRotationTarget(ArmRotations.GRAB.position);
    }

    public void fullRet() {
        setRetractionTarget(ArmRetractions.REST.position);
    }
    public void fullExtend() {
        setRetractionTarget(ArmRetractions.MAX.position);
    }
}
