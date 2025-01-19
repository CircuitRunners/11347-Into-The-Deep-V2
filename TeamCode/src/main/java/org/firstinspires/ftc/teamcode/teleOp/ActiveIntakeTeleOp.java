package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.activeIntake;

@Config
@TeleOp
public class ActiveIntakeTeleOp extends CommandOpMode {

    private activeIntake intakeSubsystem;
    public static double pivPos = 0, holdPos = 0, power = 0;

    @Override
    public void initialize() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the subsystem
        intakeSubsystem = new activeIntake(hardwareMap);

        telemetry.addLine("Active Intake TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        intakeSubsystem.setPivot(pivPos);
        intakeSubsystem.setHold(holdPos);
        

        telemetry.addData("pivot", pivPos);
        telemetry.addData("hold", holdPos);
        telemetry.update();
    }
}
