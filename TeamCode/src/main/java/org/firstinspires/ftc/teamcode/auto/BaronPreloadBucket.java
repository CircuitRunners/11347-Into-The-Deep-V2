package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name="BlueBucket", group = ".", preselectTeleOp = "TeleOp")
public class BaronPreloadBucket extends OpMode {
    public int pathState;
    public Auto auto;

    public Timer pathTimer = new Timer();

    @Override
    public void init() {
        auto = new Auto(hardwareMap, telemetry, new Follower(hardwareMap), false, true);
    }

    @Override
    public void start() {
        auto.start();
        setPathState(0);
    }

    @Override
    public void loop() {
        telemetry.addData("State: ", pathState);
        telemetry.addData("Path Timer: ", pathTimer.getElapsedTimeSeconds());
        auto.update();
        pathUpdate();

        telemetry.update();
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                auto.rotPID = false;
                auto.retPID = false;
                auto.startScoring();
                setPathState(999);
                break;

            case 999:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    auto.follower.setMaxPower(1);
                    auto.follower.followPath(auto.preloadPark, false);
                    setPathState(1);
                }
                break;

            case 1:
                if (auto.actionNotBusy()) {
                    auto.arm.toHighBucket();
                }
                break;
        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}
