package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends CommandOpMode {

    // Hardware references
    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;

    // Gamepad snapshots
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void initialize() {
        // Set up bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        // Pinpoint driver initialization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        telemetry.addLine("Ready!");
        telemetry.update();
    }




    @Override
    public void run() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        double forward = -currentGamepad1.left_stick_y;
        double strafe  =  currentGamepad1.left_stick_x;
        double rotate  =  currentGamepad1.right_stick_x;

        Pose2D currentPose = driveFieldRelative(forward, strafe, rotate);

        // If x is pressed, reset IMU
        if (currentGamepad1.square) {
            pinpoint.resetPosAndIMU();
        }
        // If y is pressed, recalibrate IMU
        if (currentGamepad1.triangle) {
            pinpoint.recalibrateIMU();
        }

        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.update();
    }

    // Configure the GoBilda Pinpoint Here
    private void configurePinpoint() {
        pinpoint.setOffsets(pinpointXOffset, pinpointYOffset);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
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