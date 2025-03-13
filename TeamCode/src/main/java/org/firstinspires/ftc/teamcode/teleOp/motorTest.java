package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.support.Constants.*;

@Config
@TeleOp
public class motorTest extends CommandOpMode {
    private PIDController controller;

    public static double p = 0.0, i = 0, d = 0.0;
    public static double f = 0.0;

    private final double ticks_in_degree = 0.0521;

    public static int target = 0;

    private DcMotorEx arm_motor, arm_motor_2;

    @Override
    public void initialize() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "fish");
        arm_motor_2 = hardwareMap.get(DcMotorEx.class, "the krish");

        arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_2.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm_motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void run() {
        super.run();

        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(-power);
        arm_motor_2.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
