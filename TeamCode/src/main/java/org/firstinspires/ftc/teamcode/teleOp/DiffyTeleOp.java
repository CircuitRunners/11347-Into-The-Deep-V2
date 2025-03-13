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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private Timer pathTimer;
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
    private int positionPosition = -3;
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

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
        driver = new GamepadEx(gamepad1);
//        driver = new GamepadEx(gamepad1);
//        manipulator = new GamepadEx(gamepad2);

        pathTimer = new Timer();

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
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(() -> {
//                    diffy.subDiffy();
//                    fish = true;
//                    claw.open();
//                }));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    positionPosition = (positionPosition + 1) % 3;
                    krish = true;
                }));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    positionPositionPosition = (positionPositionPosition + 1) % 3;
                }));
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    claw.open();
                }));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    claw.close();
                }));
        
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
//                    retTarget = retTarget +500;
                    ret.reset();
                    retTarget = 0;
                }));
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whileActiveContinuous(new InstantCommand(() -> {
                    retTarget = retTarget -500;
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
//          if (gamepad1.circle) {
//            claw.open();
//        }
        // if (rightBumperJustPressed) {
        //     depositState = (depositState + 1) % 5; // cycle 0..4
        // }
        // if (rightTriggerJustPressed) {
        //     depositState = 0; // cycle 0..4
        // }
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    rotTarget= rotTarget -20;
                }));
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                .whileActiveContinuous(new InstantCommand(() -> {
                    rotTarget= rotTarget +20;
                }));
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
//        while (gamepad1.left_trigger > 0.2) {
//            rotTarget = rotTarget +15;
//        }
//        while (gamepad1.right_trigger > 0.2) {
//            rotTarget = rotTarget - 15;
//        }
        if (gamepad1.dpad_up) {
            diffy.rotateDiffyL();
        }
        if (gamepad1.dpad_down) {
            diffy.rotateDiffyR();
        }
        if (gamepad1.circle) {
            claw.close();
        }
        if (gamepad1.square){
            claw.open();
        }
        if (gamepad1.dpad_right) {
            diffy.moveDiffyP();
        }
        if (gamepad1.dpad_left) {
            diffy.moveDiffyN();
        }

//        if (gamepad1.circle && gamepad1.right_bumper) {
//            pinpoint.resetPosAndIMU();
//            diffy.centerDiffy();
//        }
        switch (positionPosition) {

            case -2:
                if (rotTarget !=1500){
                    rotTarget = 1500;
                    diffy.subDiffy();
                    claw.open();
                    positionPosition = positionPosition+1;
                }
                break;
            case 0:
                //phase = 0;
                if (retTarget <500) {
                    rotTarget = 1650; //extension = 46000
                    retTarget = 30000;
//
//                rotTarget = 1100;
//                //retTarget = 57000;
//                ret.setTarget(57000);
//                fish = true;
                    diffy.subDiffy();
                    claw.open();
                }
                pathTimer.resetTimer();
                break;
            case 1:
                if (rotTarget >1350) {
                    rotTarget = 1300;

                }
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    claw.close();
                }
                if (pathTimer.getElapsedTimeSeconds() >0.4){
                    if (retTarget >1400) {
                        retTarget = 0;
                        rotTarget = 1500;
                        diffy.centerDiffy();
                        positionPosition = -1;
                    }
                }
                break;
//            case 2:
//
//                break;
            default:
                break;
        }
        switch (positionPositionPosition) {
            case 0:
                if (retTarget <1000) {
                    rotTarget = 8300;
                    diffy.centerDiffy();
                }
                break;

            case 1:


                if (retTarget < 10000)
                {


                    retTarget = 52000;


                }
                pathTimer.resetTimer();
                break;
            case 2:
                if (retTarget >10000) {
                    diffy.endDiffy();
                    claw.open();
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        retTarget = 0;
                        diffy.centerDiffy();
                        }
                    }
                if (rotTarget >5000){
                    if (pathTimer.getElapsedTimeSeconds() > 1.7) {
                        rotTarget = 1500;

                        positionPositionPosition=-1;
                    }
                }

                break;
//            case 3:
//
//                if (ret.getCurrentRetraction() < 5000) {
//                    rotTarget = 1500;
//                    diffy.centerDiffy();
//                }
//                break;
            default:
                break;
        }
        if (gamepad1.triangle && gamepad1.right_bumper) {
            pinpoint.recalibrateIMU();
        }



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
