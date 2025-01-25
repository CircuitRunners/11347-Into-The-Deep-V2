package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.pinpointXOffset;
import static org.firstinspires.ftc.teamcode.support.Constants.pinpointYOffset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmRet;
import org.firstinspires.ftc.teamcode.subsystems.ArmRot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;

import java.util.Locale;

@Config
@TeleOp(name="RunThisOne", group=".")
public class MainTeleOp extends CommandOpMode {
    private MecanumDrive drive;
    private ArmRot rot;
    private ArmRet ret;
    private activeIntake intake;
    private GoBildaPinpointDriver pinpoint;
    private boolean isExtended = false, atRest = false, pidActive = false;

    public static int rotTarget = 0, retTarget = 0;

    GamepadEx driver, manipulator;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        driver = new GamepadEx(gamepad1);
        manipulator = new GamepadEx(gamepad2);

        // SUBSYSTEMS
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        rot = new ArmRot(hardwareMap, telemetry);
        ret = new ArmRet(hardwareMap, telemetry);
        rotTarget = 0;
        retTarget = 0;

        intake = new activeIntake(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        // TOP BUCKET
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            pidActive = true;
                            atRest = false;
                            isExtended = true;
                            intake.setTarget(300);
                            rotTarget = 1500;
                            retTarget = 57000;
                        }),
                        new InstantCommand(() -> rotTarget = 8000),
                        new WaitCommand(2), // Waits for 2 seconds
                        new InstantCommand(() -> intake.setTarget(75))
                ))
                .whenReleased(new InstantCommand(() -> {
                    pidActive = false;
                    atRest = false;
                    isExtended = true;
                }));

        // REST
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                        .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    pidActive = true;
                                    atRest = false;
                                    isExtended = true;
                                    intake.setTarget(300);
                                    retTarget = 300;
                                }),
                                new WaitCommand(1),
                                new InstantCommand(() -> rotTarget = 1500)
                        ))
                        .whenReleased(new InstantCommand(() -> {
                            pidActive = false;
                            atRest = true;
                            isExtended = false;
                        }));

        // GRAB
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    pidActive = true;
                                    atRest = false;
                                    isExtended = true;
                                    rotTarget = 1000;
                                    retTarget = 57000;
                                }),
                                new WaitCommand(2),
                                new InstantCommand(() -> intake.setTarget(100))
                        ))
                        .whenReleased(new InstantCommand(() -> {
                            pidActive = false;
                            atRest = false;
                            isExtended = true;
                        }));

        // TOP BAR
        manipulator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    pidActive = true;
                                    atRest = false;
                                    isExtended = true;
                                    intake.setTarget(300);
                                    rotTarget = 4500;
                                    retTarget = 7500;
                                }),
                                new WaitCommand(3),
                                new InstantCommand(() -> retTarget = 300)
                        ))
                        .whenReleased(new InstantCommand(() -> {
                            pidActive = false;
                            atRest = false;
                            isExtended = true;
                        }));






        telemetry.addLine("READY!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        rot.update();
        ret.update();

        rot.setTarget(rotTarget);
        ret.setTarget(retTarget);

        double forward = -driver.getLeftY();
        double strafe = -driver.getLeftX();
        double rotate = driver.getRightX();

        boolean slow = gamepad1.square;
        Pose2D currentPose;

        if (slow) {
            currentPose = driveFieldRelative(forward * 0.1, strafe * 0.1, rotate * 0.1);
        } else {
            currentPose = driveFieldRelative(forward, strafe, rotate);
        }



        if (gamepad1.circle && gamepad1.right_bumper) {
            pinpoint.resetPosAndIMU();
        }

        if (gamepad1.triangle && gamepad1.right_bumper) {
            pinpoint.recalibrateIMU();
        }












        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );


        telemetry.addData("Current Retraction", ret.getCurrentRetraction());
        telemetry.addData("Target Retraction", ret.getTarget());
        telemetry.addData("Current Rotation", rot.getCurrentRotation());
        telemetry.addData("Target Rotation", rot.getTarget());
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.update();
    }

    private void configurePinpoint() {
        pinpoint.resetPosAndIMU();

        pinpoint.setOffsets(pinpointXOffset, pinpointYOffset);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }
}
