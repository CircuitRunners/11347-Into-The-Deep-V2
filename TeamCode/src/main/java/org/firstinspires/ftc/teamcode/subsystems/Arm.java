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
public class Arm extends SubsystemBase {

    public enum ArmRotations {
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

    public int rotationTarget = targetRotation;
    public int retractionTarget = targetRetraction;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private static final double VOLTAGE_WHEN_TUNED = 13.0;

    public RunAction testDown, testUp;

    public Arm(HardwareMap hardwareMap) {
        RotationController = new PIDController(ROTp, ROTi, ROTd);
        RetractionController = new PIDController(RETp, RETi, RETd);

        // Initialize motors
        rotationOne = hardwareMap.get(DcMotorEx.class, ROTATION_ONE);
        rotationTwo = hardwareMap.get(DcMotorEx.class, ROTATION_TWO);
        retractOne = hardwareMap.get(DcMotorEx.class, RETRACTION_ONE);
        retractTwo = hardwareMap.get(DcMotorEx.class, RETRACTION_TWO);

        retractTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        resetEncoders();
        resetTargets();

        rotationTarget = getCurrentRotation();
        retractionTarget = getCurrentRetraction();

        // Initialize actions
        testDown = new RunAction(this::rest);
        testUp = new RunAction(this::max);

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
        setRotationPower(ROTpower);
        setRetractionPower(RETpower);
    }


    /** GETTER METHODS FOR POSITIONS */
    public int getCurrentRotation() {
        return rotationTwo.getCurrentPosition();
    }
    public int getCurrentRetraction() {
        return retractTwo.getCurrentPosition();
    }


    /** TARGET SETTER METHODS */
    public void setRotationTarget(int target) {
        this.rotationTarget = target;
    }
    public void setRetractionTarget(int target) {
        this.retractionTarget = target;
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
    public void rest() {
        setRotationTarget(ArmRotations.REST.position);
    }
    public void mid() {
        setRotationTarget(ArmRotations.MID.position);
    }
    public void max() {
        setRotationTarget(ArmRotations.MID.position);
    }

    public void fullRet() {
        setRetractionTarget(ArmRetractions.REST.position);
    }
    public void fullExtend() {
        setRetractionTarget(ArmRetractions.MAX.position);
    }
}
