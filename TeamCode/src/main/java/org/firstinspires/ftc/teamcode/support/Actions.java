package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.support.Action;

public class Actions {

    public static void runBlocking(Action a) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas c = new Canvas();
        a.preview(c);

        boolean b = true;
        while (b && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = a.run(p);

            dash.sendTelemetryPacket(p);
        }
    }
}
