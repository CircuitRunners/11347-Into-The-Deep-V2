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
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

import java.util.Locale;

@Config
@TeleOp(name="RunThisOneDiffy", group=".")
public class DiffyTeleOp extends CommandOpMode {
    private MecanumDrive drive;
    private ArmRot rot;
    private boolean fish = false;
    private ArmRet ret;
    private Claw claw;
    private int positionPosition = -1;
    private Diffy diffy;
    private GoBildaPinpointDriver pinpoint;
    private boolean isExtended = false, atRest = false, pidActive = false;

    public static int rotTarget = 0, retTarget = 0;
    private boolean previousLeftBumperState = false;
    private boolean IntakeBoolean = true;
    GamepadEx driver;//, previousGamepad1;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        driver = new GamepadEx(gamepad1);
//        driver = new GamepadEx(gamepad1);
//        manipulator = new GamepadEx(gamepad2);

        // SUBSYSTEMS
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        rot = new ArmRot(hardwareMap, telemetry);
        ret = new ArmRet(hardwareMap, telemetry);
        rotTarget = 0;
        retTarget = 0;
        claw = new Claw(hardwareMap);
        diffy = new Diffy(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();
//          boolean leftBumperJustPressed =
//                (currentGamepad1.left_bumper && !previousGamepad1.left_bumper);
//        boolean rightBumperJustPressed =
//                (currentGamepad1.right_bumper && !previousGamepad1.right_bumper);
//
//        boolean leftTriggerJustPressed =
//                (currentGamepad1.left_trigger > 0.5f) && (previousGamepad1.left_trigger <= 0.5f);
//        boolean rightTriggerJustPressed =
//                (currentGamepad1.right_trigger > 0.5f) && (previousGamepad1.right_trigger <= 0.5f);
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new SequentialCommandGroup(
//                        new InstantCommand(() -> diffy.centerDiffy()),
//                                new InstantCommand(() -> {
//                                    fish = true;
//                                })
//                        ));
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    diffy.subDiffy();
//                    fish = true;
//                    claw.open();
//                }));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    changeIntake();
                }));
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    diffy.subDiffy();
//                    fish = false;
//                    claw.close();
//                }));




        /** AUTOMATIONS */
//        if (leftBumperJustPressed) {
//            intakeState = (intakeState + 1) % 4; // cycle 0..3
//        }
//        boolean currentLeftBumperState = gamepad1.left_bumper;
//        if (currentLeftBumperState && !previousLeftBumperState) {
//            changeIntake();
//        }
//        previousLeftBumperState = currentLeftBumperState;

        // if (leftTriggerJustPressed) {
        //     intakeState = (intakeState - 1) % 4; // cycle 0..3
        // }
          if (gamepad1.left_trigger >0) {
            //claw.open();
        }
        // if (rightBumperJustPressed) {
        //     depositState = (depositState + 1) % 5; // cycle 0..4
        // }
        // if (rightTriggerJustPressed) {
        //     depositState = 0; // cycle 0..4
        // }
        if (gamepad1.right_trigger >0) {
            //claw.close();
        }



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

        boolean slow = gamepad1.square;
        Pose2D currentPose;

        if (slow) {
            currentPose = driveFieldRelative(forward * 0.5, strafe * 0.5, rotate * 0.5);
        } else {
            currentPose = driveFieldRelative(forward, strafe, rotate);
        }

        while (gamepad1.dpad_up) {
            diffy.rotateDiffyL();
        }
        while (gamepad1.dpad_down) {
            diffy.rotateDiffyR();
        }

//        if (gamepad1.circle && gamepad1.right_bumper) {
//            pinpoint.resetPosAndIMU();
//            diffy.centerDiffy();
//        }


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
        telemetry.addData("krish = ", diffy.leftPosition());
        telemetry.addData("fishy = ", diffy.rightPosition());
        telemetry.addData("Ez", fish);
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
  public void changeIntake() {
        if (IntakeBoolean) {
            switch (positionPosition) {
                case 0:
                    rotTarget = 900;
                    //retTarget = 57000;
                    fish = true;
                    diffy.centerDiffy();
                    positionPosition = 1;
                    break;

                case 1:
                    //retTarget = 300;
                    diffy.subDiffy();
                    //if
                    // chage: new InstantCommand(() -> rotTarget = 1500);
                    positionPosition = 0;
                    
                    IntakeBoolean = false;
                    break;

            }
        } else {
            switch (positionPosition) {
                case 0:
                    rotTarget = 900;
                    //retTarget = 57000;

                    diffy.centerDiffy();
                    positionPosition = 1;
                    IntakeBoolean = true;
                    break;
            }
        }

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
