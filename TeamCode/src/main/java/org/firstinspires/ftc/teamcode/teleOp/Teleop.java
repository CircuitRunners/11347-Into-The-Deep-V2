package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmRet;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;

@Config
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends CommandOpMode {

    // Hardware references
    private MecanumDrive drive;
    private Arm arm;

    private ArmRet ret;
    private activeIntake intake;
    private GoBildaPinpointDriver pinpoint;

    public int retractionTarget = 0;
    public int rotationTarget = 300;

    public static int target = 0, MIN = 0;
    public static int testMult = 105;

    GamepadEx driver, manipulator;

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));
         driver = new GamepadEx(gamepad1);
         manipulator = new GamepadEx(gamepad2);

        // Initialize subsystems
        drive = new MecanumDrive();
        intake = new activeIntake(hardwareMap);
        arm = new Arm(hardwareMap);
        ret = new ArmRet(hardwareMap, telemetry);
        arm.resetTargets();
        drive.init(hardwareMap);

        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
//        arm.update();
//        ret.update();

        double forward = -driver.getLeftY();
        double strafe  =  -driver.getLeftX();
        double rotate  =  driver.getRightX();

        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // If x is pressed, reset IMU
        if (gamepad1.square) {
            pinpoint.resetPosAndIMU();
        }
        // If y is pressed, recalibrate IMU
        if (gamepad1.triangle) {
            pinpoint.recalibrateIMU();
        }

//        // RETRACTION
//        if (gamepad2.square) {
//            retractionTarget = Arm.ArmRetractions.MID.getPosition();
//        }
//        if (gamepad2.circle) {
//            retractionTarget = Arm.ArmRetractions.REST.getPosition();
//        }
//
//
//        // ROTATION
////        if (gamepad2.dpad_up) {
////            rotationTarget = Arm.ArmRotations.MAX.getPosition();
////        }
//        if (gamepad2.dpad_left) {
//            target = -57000;
//        }
//        if (gamepad2.dpad_down) {
//            target = -300;
//        }

//        ret.setTarget(target);
//        ret.manual(gamepad2.left_stick_y);

//        if (gamepad2.left_stick_x > 0.3 || gamepad2.left_stick_x < 0.3) {
//            rotationTarget += ((int) gamepad2.left_stick_x) * testMult;
//        }

//        arm.setRetractionTarget(retractionTarget);
        arm.setRotationTarget(rotationTarget);

        // Intake
        intake.intake(gamepad2.left_trigger - gamepad2.right_trigger);

        if (gamepad2.left_bumper) {
            intake.holdOpen();
        }
        if (gamepad2.right_bumper) {
            intake.holdClosed();
        }

//        intake.setPivot(gamepad2.right_stick_y);


        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        telemetry.addData("Current Retraction", arm.getCurrentRetraction());
        telemetry.addData("Target Retraction", ret.getTarget());
        telemetry.addData("Current Rotation", arm.getCurrentRotation());
        telemetry.addData("Target Rotation", rotationTarget);
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.update();
    }

    // Configure the GoBilda Pinpoint Here
    private void configurePinpoint() {
//        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        pinpoint.setOffsets(pinpointXOffset, pinpointYOffset);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    // ------------------------------------------------
    // HELPER: Field-relative drive
    // ------------------------------------------------
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
