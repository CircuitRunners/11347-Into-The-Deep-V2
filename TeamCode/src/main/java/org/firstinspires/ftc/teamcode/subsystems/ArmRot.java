package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.support.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmRot {
    public enum ArmRotations {
        DRIVE(300),
        GRAB(150),
        REST(500),
        MID(2000),
        MAX(8500);

        public int position;

        ArmRotations(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    private PIDController controller;
    public static double p = 0.0022, i = 0, d = 0.00015;
    public static double f = 0.17;

    public static int target = 0;

    private DcMotorEx armMotor1, armMotor2;

    public Telemetry telemetry;

    public boolean manual = false;

    public ArmRot(HardwareMap hardwareMap, Telemetry telemetry) {
        controller = new PIDController(p, i, d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor1 = hardwareMap.get(DcMotorEx.class, ROTATION_ONE);
        armMotor2 = hardwareMap.get(DcMotorEx.class, ROTATION_TWO);

        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        if (!manual) {
            controller.setPID(p, i, d);
            int armPos = armMotor2.getCurrentPosition();
            double pid = controller.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(target)) * f;

            double power = pid + ff;

            armMotor1.setPower(-power);
            armMotor2.setPower(-power);

//            telemetry.addData("ROT POS: ", armPos);
//            telemetry.addData("ROT TARGET: ", target);
//            telemetry.update();
        }
    }

    public int getCurrentRotation() {
        return armMotor2.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }

    public void setTarget(int target) {
        ArmRot.target = target;
        manual = false;
    }

    public void manual(double n) {
        n = -n;
        manual = true;

        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor1.setPower(n);
        armMotor2.setPower(n);
    }

    public void reset() {
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTarget(0);
    }
}
