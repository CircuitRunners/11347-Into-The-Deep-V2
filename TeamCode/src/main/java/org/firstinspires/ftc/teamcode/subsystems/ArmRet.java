package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmRet {
    public enum ArmRetractions {
        REST(300),
        MID(30000),
        MAX(57000);

        public int position;

        ArmRetractions(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }
    }

    private PIDController controller;
    public static double p = 0.001, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private DcMotorEx armMotor3, armMotor4;

    public Telemetry telemetry;

    public boolean manual = false;

    public ArmRet(HardwareMap hardwareMap, Telemetry telemetry) {
        controller = new PIDController(p, i, d);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor3 = hardwareMap.get(DcMotorEx.class, "leftRetraction");
        armMotor4 = hardwareMap.get(DcMotorEx.class, "rightRetraction");

        armMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        if (!manual) {
            controller.setPID(p, i, d);
            int armPos = armMotor4.getCurrentPosition();
            double pid = controller.calculate(armPos, target);

            double ff = Math.cos(Math.toRadians(target)) * f;

            double power = pid + ff;

            armMotor3.setPower(-power);
            armMotor4.setPower(-power);

//            telemetry.addData("RET POS: ", armPos);
//            telemetry.addData("RET TARGET: ", target);
//            telemetry.update();
        }
    }

    public int getCurrentRetraction() {
        return armMotor4.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }

    public void setTarget(int target) {
        ArmRet.target = target;
        manual = false;
    }

    public void manual(double n) {
        n = n;
        manual = true;

        armMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor3.setPower(n);
        armMotor4.setPower(n);
    }

    public void reset() {
        armMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        target = 0;
    }
}
