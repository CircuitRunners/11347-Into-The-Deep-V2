package org.firstinspires.ftc.teamcode.subsystems.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends OpMode {

    private Servo testServo;
    private double position = 0.0;
    private final double INCREMENT = 0.01; // Amount to increment servo position
    private final double MAX_POSITION = 1.0;
    private final double MIN_POSITION = 0.0;

    @Override
    public void init() {
        // Initialize the servo (name it "testServo" in your hardware configuration)
        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setPosition(position);
        telemetry.addData("ServoTester", "Initialized");
    }

    @Override
    public void loop() {
        // Increment the servo position when the "A" button is pressed
        if (gamepad1.a && position < MAX_POSITION) {
            position += INCREMENT;
            if (position > MAX_POSITION) {
                position = MAX_POSITION;
            }
        }

        // Decrement the servo position when the "B" button is pressed
        if (gamepad1.b && position > MIN_POSITION) {
            position -= INCREMENT;
            if (position < MIN_POSITION) {
                position = MIN_POSITION;
            }
        }

        // Update the servo position
        testServo.setPosition(position);

        // Display the current servo position
        telemetry.addData("Servo Position", "%.2f", position);
        telemetry.update();
    }
}
