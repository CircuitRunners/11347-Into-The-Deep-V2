package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class NullAction implements Action {
    @Override
    public boolean run(TelemetryPacket p) {
        return false;
    }
}