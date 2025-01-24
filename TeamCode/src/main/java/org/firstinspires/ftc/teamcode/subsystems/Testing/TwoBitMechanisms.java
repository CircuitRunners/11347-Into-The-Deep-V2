package org.firstinspires.ftc.teamcode.subsystems.Testing;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.support.PID;

public class TwoBitMechanisms extends SubsystemBase {
    final private DcMotor rot1, rot2, ret1, ret2;

    PID pid = new PID(0.0015, 0, 0.00015);
    PID ret = new PID(0.0003, 0, 0);

    public TwoBitMechanisms(DcMotor rot1, DcMotor rot2, DcMotor ret1, DcMotor ret2) {
        this.rot1 = rot1;
        this.rot2 = rot2;

        rot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.ret1 = ret1;
        this.ret2 = ret2;

        ret1.setDirection(DcMotorSimple.Direction.REVERSE);

        ret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ret1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ret2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runManual(double rotp, double retp) {
        rot1.setPower(-rotp - getPower());
        rot2.setPower(-rotp - getPower());

        ret1.setPower(-retp);
        ret2.setPower(-retp);
    }

    public double getPower() {
        double power;
        double current = getPosition();
        if (current > 8000) {
            power = 0;
        } else if (current < 300) {
            power = 0;
        } else {
            power = 0.05;
        }

        return power;
    }

    public void runPID(double target) {
        double command = pid.update(rot2.getCurrentPosition(), target) - 0.17;

        rot1.setPower(-command);
        rot2.setPower(-command);
    }

    public double getPosition() {
        return rot2.getCurrentPosition();
    }



    public void retPID(double target) {
        double command = pid.update(ret2.getCurrentPosition(), target);

        ret1.setPower(command);
        ret2.setPower(command);
    }

    public void resetRot() {
        rot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRet() {
        ret1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getRet() {
        return ret2.getCurrentPosition();
    }
}
