package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.support.Encoder;
import org.firstinspires.ftc.teamcode.support.RunAction;
import static org.firstinspires.ftc.teamcode.support.Constants.*;

@Config
public class Arm extends SubsystemBase {
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

    public double rotationTarget = targetRotation;
    public double retractionTarget = targetRetraction;

    private Encoder rotEnc, retEnc;

    private VoltageSensor voltageSensor;
    private double voltageComp;
    private double VOLTAGE_WHEN_TUNED = 13.0;

    public RunAction testDown, testUp;

    public Arm(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        rotationOne = hardwareMap.get(DcMotorEx.class, ROTATION_ONE);
        rotationTwo = hardwareMap.get(DcMotorEx.class, ROTATION_TWO);
        retractOne = hardwareMap.get(DcMotorEx.class, RETRACTION_ONE);
        retractTwo = hardwareMap.get(DcMotorEx.class, RETRACTION_TWO);

        // Initialize Encoders Here
        rotEnc = Encoder(hardwareMap.get(DcMotorEx.class, ROTATION_ONE));
        retEnc = Encoder(hardwareMap.get(DcMotorEx.class, RETRACTION_ONE));

        // Reverse encoders as needed
        // rotEnc.setDirection(Encoder.Direction.REVERSE);
        // retEnc.setDirection(Encoder.Direction.REVERSE);

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

    /* GETTER METHODS FOR POSITIONS */
    public int getCurrentRotation() {
        return rotEnc.getCurrentPosition();
    }
    public int getCurrentRetraction() {
        return retEnc.getCurrentPosition();
    }

    /* TARGET SETTER METHODS */
    public void setRotationTarget(double target) {
        this.rotationTarget = target;
    }
    public void setRetractionTarget(double target) {
        this.retractionTarget = target;
    }

    /* GETTER METHODS FOR ARM CURRENTS */
    public double getArmCurrent() {
        return rotationOne.getCurrent(CurrentUnit.AMPS);
    }
    public double getArmCurrent() {
        return retractOne.getCurrent(CurrentUnit.AMPS);
    }

    /* TARGET GETTER METHODS */
    public double getRotationTarget() {
        return rotationTarget;
    }
    public double getRetractionTarget() {
        return retractionTarget;
    }

    /* MANUAL CONTROL METHODS */
    public void manualRotaion(double n) {
        setRotationTarget(rotationTarget + n * ARM_SPEED);
    }
    public void manualRetraction(double n) {
        setRetractionTarget(retractionTarget + n * RETRACT_SPEED);
    }

    /* ROTATION COMMANDS */
    public void testDown() {
        setRotationTarget(ArmRotations.REST.position);
    }
    public void testUp() {
        setRotationTarget(ArmRotations.MID.position);
    }

    /* RETRACTION COMMANDS */
    // TODO: ADD COMMANDS HERE

    /* RESET ALL ENCODERS */
    // TODO: CHECK IF THIS CAN BE REMOVED
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

    /* RESET ROTATION TARGET & ENCODER */
    public void resetRotationPosition() {
        setRotationTarget(0);
        rotationOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotationOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotationTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* RESET RETRACTION TARGET & ENCODER */
    public void resetRetractionPosition() {
        setRetractionTarget(0);
        retractOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        retractTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        retractOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* GETTER METHODS FOR ENC VELOCITIES */
    public double getRotationVelocity() {
        return rotEnc.getCorrectedVelocity();
    }
    public double getRetractionVelocity() {
        return retEnc.getCorrectedVelocity();
    }

    /* RETURNS VOLTAGECOMP */
    public double getVoltageComp() {
        return voltageComp;
    }

    /* SET POWER METHODS */
    public void setRotationPower(double power) {
        rotationOne.setPower(power);
        rotationTwo.setPower(power);
    }
    public void setRetractionPower(double power) {
        retractOne.setPower(power);
        retractTwo.setPower(power);
    }
    public void setAllPower(double power) {
        rotationOne.setPower(power);
        rotationTwo.setPower(power);

        retractOne.setPower(power);
        retractTwo.setPower(power);
    }

    /* BRAKE */
    public void brake() {
        setAllPower(0);
    }
}
