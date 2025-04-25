package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.pinpointXOffset;
import static org.firstinspires.ftc.teamcode.support.Constants.pinpointYOffset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
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
@TeleOp(name="MainTeleOp", group=".")
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



        /** AUTOMATIONS */
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
                        new WaitCommand(2000), // Waits 2 seconds
                        new InstantCommand(() -> rotTarget = 8000),
                        new WaitCommand(500),
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
                                new WaitCommand(1000), // Waits 1 second
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
                                new InstantCommand(() -> intake.setTarget(85)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> {
                                    pidActive = true;
                                    atRest = false;
                                    isExtended = true;
                                    rotTarget = 900;
                                    retTarget = 57000;
                                }),
                                new WaitCommand(2000) // Waits 2 seconds
//                                new InstantCommand(() -> intake.setTarget(135))
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
                                    intake.setTarget(200);
                                    rotTarget = 4500;
                                    retTarget = 7700;
                                }),
                                new WaitCommand(3000), // Waits 3 seconds
                                new InstantCommand(() -> intake.setTarget(65)),
                                new InstantCommand(() -> retTarget = 300)
                        ))
                        .whenReleased(new InstantCommand(() -> {
                            pidActive = false;
                            atRest = false;
                            isExtended = true;
                        }));



        /** ARM MANUAL STUFF */
        // MANUAL ROTATION
        new Trigger(() -> manipulator.getLeftX() > 0.1 || manipulator.getLeftX() < -0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    double power = manipulator.getLeftX();

                    rot.manual(power);
                }));

        // MANUAL RETRACTION
        new Trigger(() -> manipulator.getLeftY() > 0.1 || manipulator.getLeftY() < -0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    double power = manipulator.getLeftY();

                    ret.manual(power);
                }));

        manipulator.getGamepadButton(GamepadKeys.Button.X)
                        .whenPressed(new InstantCommand(() -> {
                            rot.reset();
                            ret.reset();
                        }));


        /** INTAKE STUFF -- RELEASE LEFT, GRAB RIGHT */
        // HOLD CLOSED
        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    intake.holdClosed();
                }));

        // HOLD OPEN
        manipulator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    intake.holdOpen();
                }));

        // INTAKE/OUTTAKE
        new Trigger(() -> manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 ||
                manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    double leftTrigger = manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                    double rightTrigger = manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

                    // Update intake power based on triggers
                    intake.intake(rightTrigger - leftTrigger);
                }))
                .whenInactive(new InstantCommand(() -> {
                    // Stop the intake when neither trigger is pressed
                    intake.intake(0);
                }));


        // PIVOT ROTATE
        new Trigger(() -> Math.abs(manipulator.getRightY()) > 0.2) // Dead zone check
                .whileActiveContinuous(new InstantCommand(() -> {
                    // Dynamically update pivot rotation based on joystick value
                    double power = manipulator.getRightY();

                    intake.setPivot(power);
                }))
                .whenInactive(new InstantCommand(() -> {
                    // Stop pivot rotation when joystick is released
                    intake.setPivot(0);
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

        double forward = driver.getLeftY();
        double strafe = driver.getLeftX();
        double rotate = driver.getRightX();

        Pose2D currentPose;

        currentPose = driveFieldRelative(forward, strafe, rotate);


        if (gamepad1.circle && gamepad1.right_bumper) {
            pinpoint.resetPosAndIMU();
        }

        if (gamepad1.triangle && gamepad1.right_bumper) {
            pinpoint.recalibrateIMU();
        }

//        intake.intake(manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        intake.setPivot(manipulator.getRightY());




        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );


        telemetry.addData("Battery Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
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
