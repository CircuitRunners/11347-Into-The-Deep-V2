package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.pinpointXOffset;
import static org.firstinspires.ftc.teamcode.support.Constants.pinpointYOffset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmRet;
import org.firstinspires.ftc.teamcode.subsystems.ArmRot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config
@TeleOp(name="forkids", group=".")
public class littlekidteleop extends CommandOpMode {
    private MecanumDrive drive;
    private ArmRot rot;
    private boolean fish = false;
    private boolean krish = false;
    private ArmRet ret;

    double leftStickYVal;
    double leftStickXVal;
    double rightStickXVal;
    double rightStickYVal;


    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    public double speedMultiply = 1;

    private double powerThreshold = 0;
    private Claw claw;
    private int positionPosition = -1;
    private int positionPositionPosition = -1;
    private Diffy diffy;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();
    private GoBildaPinpointDriver pinpoint;
    private boolean isExtended = false, atRest = false, pidActive = false;

    public static int rotTarget = 0, retTarget = 0;
    private boolean previousLeftBumperState = false;
    private boolean IntakeBoolean = true;
    GamepadEx driver;//, previousGamepad1;
    GamepadEx manipulator;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        driver = new GamepadEx(gamepad1);
//        driver = new GamepadEx(gamepad1);
//        manipulator = new GamepadEx(gamepad2);

        // SUBSYSTEMS
        drive = new MecanumDrive();
        manipulator = new GamepadEx(gamepad2);
        drive.init(hardwareMap);

        rot = new ArmRot(hardwareMap, telemetry);
        ret = new ArmRet(hardwareMap, telemetry);
        rotTarget = 0;
        retTarget = 0;
        claw = new Claw(hardwareMap);
        diffy = new Diffy(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    diffy.subDiffy();
//                    fish = true;
//                    claw.open();
//                }));

        manipulator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    positionPosition = (positionPosition + 1) % 5;
                    krish = true;
                }));
        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    positionPositionPosition = (positionPositionPosition + 1) % 5;
                }));
        manipulator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    claw.open();
                }));
        manipulator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    claw.close();
                }));
        manipulator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    diffy.subDiffy();;
                }));
        manipulator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    diffy.centerDiffy();;
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
          if (gamepad1.circle) {
            claw.open();
        }
        // if (rightBumperJustPressed) {
        //     depositState = (depositState + 1) % 5; // cycle 0..4
        // }
        // if (rightTriggerJustPressed) {
        //     depositState = 0; // cycle 0..4
        // }
        if (gamepad1.square) {
            claw.close();
        }

//        new Trigger(() -> driver.getLeftY() > 0.1 || driver.getLeftY() < -0.1)
//                .whileActiveContinuous(new InstantCommand(() -> {
//                    double power = driver.getLeftY();
//
//                    ret.manual(power);
//                }));
//
//        driver.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new InstantCommand(() -> {
//                    rot.reset();
//                    ret.reset();
//                }));



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


        leftStickYVal = gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = -gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = -gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

        frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

        rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

        rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
            frontLeftSpeed = 0;
            drive.frontLeftMotor.setPower(frontLeftSpeed);
        } else {
            drive.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
        }

        if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
            frontRightSpeed = 0;
            drive.frontRightMotor.setPower(frontRightSpeed);
        } else {
            drive.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
        }

        if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
            rearLeftSpeed = 0;
            drive.backLeftMotor.setPower(rearLeftSpeed);
        } else {
            drive.backLeftMotor.setPower(rearLeftSpeed * speedMultiply);
        }

        if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
            rearRightSpeed = 0;
            drive.backRightMotor.setPower(rearRightSpeed);
        } else {
            drive.backRightMotor.setPower(rearRightSpeed * speedMultiply);
        }

//        if (slow) {
//            currentPose = driveFieldRelative(forward * 0.5, strafe * 0.5, rotate * 0.5);
//        } else {
//            currentPose = driveFieldRelative(forward, strafe, rotate);
//        }

        if (positionPosition !=-1 && positionPositionPosition !=-1){
            positionPosition =-1;
            positionPositionPosition = -1;
        }
        if (gamepad2.left_trigger > 0.2) {
            //rot.manual(gamepad2.left_trigger);
            rotTarget= rotTarget+10;
        }
        if (gamepad2.right_trigger > 0.2) {
            //rot.manual(-gamepad2.right_trigger);
            rotTarget = rotTarget-10;
        }
        if (gamepad2.dpad_up) {
            diffy.rotateDiffyL();
        }
        if (gamepad2.dpad_down) {
            diffy.rotateDiffyR();
        }
        if (gamepad2.dpad_right) {
            diffy.moveDiffyN();
        }
        if (gamepad2.dpad_left) {
            diffy.moveDiffyP();
        }

//        if (gamepad1.circle && gamepad1.right_bumper) {
//            pinpoint.resetPosAndIMU();
//            diffy.centerDiffy();
//        }
        switch (positionPosition) {
//            case -1:
//                rotTarget = 0;
//                retTarget = 0;
//                break;
            case 0:
              rotTarget = 1500; //extension = 46000
              retTarget = 30000;
//
//                rotTarget = 1100;
//                //retTarget = 57000;
//                ret.setTarget(57000);
//                fish = true;
                diffy.subDiffy();
                claw.open();
                break;
            case 1:
                rotTarget = 1380;
                fish = false;
                break;
            case 2:
                rotTarget=1340;
            case 3:
                claw.close();

                break;
            case 4:
                retTarget=0;
                rotTarget = 1500;
                break;
            default:
                break;
        }
        switch (positionPositionPosition) {
            case 0:
                rotTarget = 1500;
                diffy.centerDiffy();

                break;

            case 1:
                rotTarget = 8300;

                if (rot.getCurrentRotation() >1400)
                {
                    diffy.endDiffy();
                    retTarget = 55000;

                    outtakeTimer.reset();
                }
                break;
            case 2:
                claw.open();
                break;
            case 3:
                retTarget = 0;
                if (ret.getCurrentRetraction() < 50) {
                    rotTarget = 1500;
                }
                break;
            default:
                break;
        }
        if (gamepad1.triangle && gamepad1.right_bumper) {
            pinpoint.recalibrateIMU();
        }

//        intake.intake(manipulator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - manipulator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        intake.setPivot(manipulator.getRightY());


//
//        String data = String.format(Locale.US,
//                "{X: %.3f, Y: %.3f, H: %.3f}",
//                currentPose.getX(DistanceUnit.INCH),
//                currentPose.getY(DistanceUnit.INCH),
//                currentPose.getHeading(AngleUnit.DEGREES)
//        );

        telemetry.addData("Battery Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Current Retraction", ret.getCurrentRetraction());
        telemetry.addData("Target Retraction", ret.getTarget());
        telemetry.addData("krish = ", diffy.leftPosition());
        telemetry.addData("fishy = ", diffy.rightPosition());
        telemetry.addData("Ez", fish);
        telemetry.addData("pz", krish);
        telemetry.addData("Current Rotation", rot.getCurrentRotation());
        telemetry.addData("Target Rotation", rot.getTarget());
//        telemetry.addData("Position", data);
        telemetry.addData("Positionposition", positionPosition);
        telemetry.addData("Positionpositionposition", positionPositionPosition);
        telemetry.addData("tenz", outtakeTimer.milliseconds());
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
 // public void changeIntake() {
        //if (IntakeBoolean) {

        //}
//        else {
//            switch (positionPosition) {
//                case 0:
//                    rotTarget = 900;
//                    //retTarget = 57000;
//
//                    diffy.centerDiffy();
//                    positionPosition = 1;
//                    IntakeBoolean = true;
//                    break;
//            }
//        }


    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();  // Current position

        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit
                .normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight   = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }
}
