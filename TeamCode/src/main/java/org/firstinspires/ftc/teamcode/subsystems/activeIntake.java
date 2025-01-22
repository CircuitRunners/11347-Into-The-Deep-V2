package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.support.Constants.*;

@Config
public class activeIntake extends SubsystemBase {

    public enum pivotPositions {
        MAX_DOWN(300),
        MID(127.5),
        MAX_UP(75);

        public double position;

        pivotPositions(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position / 360;
        }
    }

    // DECLARE SERVOS
    private CRServo leftIntake, rightIntake;
    private Servo pivotServo, hold;

    private double defaultPivot = 127.5;
    private ColorSensor colorSensor;

    // Timer for intake
    private long intakeStartTime = 0;
    private static final long TIMEOUT_MS = 5000;

    public activeIntake(HardwareMap hardwareMap) {
        // CRSERVOS
        leftIntake = hardwareMap.get(CRServo.class, LEFT_INTAKE);
        rightIntake = hardwareMap.get(CRServo.class, RIGHT_INTAKE);

        // STANDARD SERVOS
        pivotServo = hardwareMap.get(Servo.class, PIVOT);
        hold = hardwareMap.get(Servo.class, HOLD);

        pivotServo.setPosition(pivotPositions.MAX_DOWN.getPosition());

        // COLOR SENSOR
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    /** POWER SETTERS FOR ACTIVE INTAKE */
    public void setIntake(double power) {
        long currentTime = System.currentTimeMillis();

        // Check if a timeout has been set or color is detected
        if (intakeStartTime == 0) {
            intakeStartTime = currentTime; // Start the timer
        }

        if (currentTime - intakeStartTime <= TIMEOUT_MS && !isColorDetected()) {
            leftIntake.setPower(power);
            rightIntake.setPower(power);
        } else {
            stop(); // Stop intake if time has exceeded or a color is detected
            intakeStartTime = 0; // Reset the timer
        }
    }

    public void intake(double power) {
        setIntake(power); // Negative power for intake
    }

    public void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    /** SET PIVOT POSES */
    public void setPivot(double pos) {
        pivotServo.setPosition(getPivot() + ((pos * 10) / 360));
    }
    public void pivotMin() {
        pivotServo.setPosition(pivotPositions.MAX_DOWN.position);
    }
    public void pivotMax() {
        pivotServo.setPosition(pivotPositions.MAX_UP.position);
    }
    public void pivotMid() {
        pivotServo.setPosition(pivotPositions.MID.position);
    }

    /** GET PIVOT POS */
    public double getPivot() {
        return pivotServo.getPosition();
    }

    /** SET HOLD POSES */
    public void setHold(double pos) {
        double posCorrected =  (pos / 360.0);
        hold.setPosition(posCorrected);
    }

    public void holdOpen() {
        double posCorrected = 120 / 360.0;
        hold.setPosition(posCorrected);
    }

    public void holdClosed() {
        double posCorrected = 190 / 360.0;
        hold.setPosition(posCorrected);
    }

    /** COLOR DETECTION LOGIC */
    private boolean isColorDetected() {
        // Read RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Thresholds for detecting red, yellow, or blue
        boolean isRed = red > 100 && green < 50 && blue < 50;
        boolean isYellow = red > 100 && green > 100 && blue < 50;
        boolean isBlue = blue > 100 && red < 50 && green < 50;

        return isRed || isYellow || isBlue;
    }

    public String logColorValues() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        return ("Red: " + red + ", Green: " + green + ", Blue: " + blue);
    }
}
