package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.support.Constants.pinpointXOffset;
import static org.firstinspires.ftc.teamcode.support.Constants.pinpointYOffset;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Testing.ArmPID;
import org.firstinspires.ftc.teamcode.subsystems.Testing.ArmRetPID;
import org.firstinspires.ftc.teamcode.subsystems.Testing.TwoBitMechanisms;

import java.util.Locale;

@TeleOp
public class CommandTeleOp extends CommandOpMode {
    TwoBitMechanisms mechanisms;
    MecanumDrive drive;
    GoBildaPinpointDriver pinpoint;

    final int HIGH = 5000;
    final int EXTEND = 57000;

    GamepadEx driver, mechanism;

    private boolean pidActive = false;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        mechanism = new GamepadEx(gamepad2);


        DcMotor rot1 = hardwareMap.dcMotor.get("backArmRotation");
        DcMotor rot2 = hardwareMap.dcMotor.get("frontArmRotation");
        DcMotor ret1 = hardwareMap.dcMotor.get("leftRetraction");
        DcMotor ret2 = hardwareMap.dcMotor.get("rightRetraction");

        drive = new MecanumDrive();
        mechanisms = new TwoBitMechanisms(rot1, rot2, ret1, ret2);
        drive.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        configurePinpoint();

        schedule(new BulkCacheCommand(hardwareMap));

        mechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {pidActive = true;}))
                .whenPressed(new ArmPID(mechanisms, HIGH).withTimeout(1500))
                .whenReleased(new InstantCommand(() -> {pidActive = false;}));

        mechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {pidActive = true;}))
                .whenPressed(new ArmRetPID(mechanisms, EXTEND).withTimeout(2500))
                .whenReleased(new InstantCommand(() -> {pidActive = false;}));


        telemetry.addLine("Init Done");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        Pose2D currentPose = driveFieldRelative(-driver.getLeftY(), -driver.getLeftX(), driver.getRightX());

        if (gamepad1.square) {
            pinpoint.resetPosAndIMU();
        }

        if(gamepad1.triangle) {
            pinpoint.recalibrateIMU();
        }

        if (!pidActive) {
            mechanisms.runManual(gamepad2.left_stick_x, gamepad2.right_stick_y);
        }



        String data = String.format(Locale.US,
                "{X: %.3f, Y: %.3f, H: %.3f}",
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        telemetry.addLine("Arm ROT: " + mechanisms.getPosition());
        telemetry.addLine("Arm RET: " + mechanisms.getRet());
        telemetry.addLine("Ret Power: " + gamepad2.right_stick_y);
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.update();
    }

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
