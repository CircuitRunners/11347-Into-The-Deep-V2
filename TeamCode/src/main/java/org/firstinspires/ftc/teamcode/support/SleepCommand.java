package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.util.Action;

public class SleepCommand implements Action {
    private final double dt;
    private double beginTs = -1;

    public SleepCommand(double dt) {
        this.dt = dt;}

    public static double now() {
        return System.nanoTime() * 1e-9;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        double t = beginTs < 0 ? 0 : now() - beginTs;
        if (beginTs < 0) {
            beginTs = now();
        }
        return t < dt;
    }
}
