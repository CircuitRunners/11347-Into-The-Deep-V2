package org.firstinspires.ftc.teamcode.teleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name="diffytest", group=".")
public class DiffyPID extends OpMode {

    private PIDController controller;
    public static double p =0, i=0, d=0;

    public static double fL = 0, fR = 0;

    public static double targetRight = 0.0;
    public static double targetLeft = 0.0;

//    private final double ticks_in_degree = 700 / 100.0;
    CRServo leftDiffyServo, rightDiffyServo;

    AnalogInput leftAxonEncoder, rightAxonEncoder;

    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftDiffyServo = hardwareMap.get(CRServo.class, "leftDiffyServo");
        rightDiffyServo = hardwareMap.get(CRServo.class, "rightDiffyServo");

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        //double leftClawPos = leftAxonEncoder.getVoltage() / 3.3 * 360;
        //double rightClawPos = rightAxonEncoder.getVoltage() / 3.3 * 360;
        double leftpid = controller.calculate(leftClawPos, targetLeft);
        double rightpid = controller.calculate(rightClawPos, targetRight);

        double ffL = Math.cos(Math.toRadians(targetLeft)) * fL;
        double ffR = Math.cos(Math.toRadians(targetRight)) * fR;

        double leftpower = leftpid + ffL;
        double rightpower = rightpid + ffR;


        leftDiffyServo.setPower(leftpower);
        rightDiffyServo.setPower(rightpower);

        //telemetry.addData("Left Pos>", leftAxonEncoder);
        //telemetry.addData("Right Pos>", rightAxonEncoder);
        telemetry.addData("target Left>", targetLeft);
        telemetry.addData("target Right>", targetRight);
        telemetry.update();
    }
}
