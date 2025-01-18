package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Config
@TeleOp(name = "Retraction Tester", group = "Testing")
public class retractionTester extends CommandOpMode {

    private Arm arm;
    public static int retractionTarget = 0;
    public static int rotationTarget = 0;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        telemetry.setMsTransmissionInterval(11);

        telemetry.addLine("Initializing Retraction Tester...");
        telemetry.update();

        // Initialize the Arm subsystem
        arm = new Arm(hardwareMap);

        // Attach the FTC Dashboard for live PID tuning
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(arm::resetEncoders));

        // Schedule the Arm subsystem update
//        schedule(new InstantCommand(arm::update));

        telemetry.addLine("Retraction Tester Ready!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        arm.update();

        arm.setRetractionTarget(retractionTarget);
        arm.setRotationTarget(rotationTarget);

        // Display live encoder and target values
        telemetry.addData("Current Retraction", arm.getCurrentRetraction());
        telemetry.addData("Target Retraction", retractionTarget);
        telemetry.addData("Current Rotation", arm.getCurrentRotation());
        telemetry.addData("Target Rotation", rotationTarget);
        telemetry.update();
    }
}
