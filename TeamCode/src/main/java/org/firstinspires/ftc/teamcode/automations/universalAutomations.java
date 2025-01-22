package org.firstinspires.ftc.teamcode.automations;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;
import org.firstinspires.ftc.teamcode.support.RunAction;

public class universalAutomations {
    public Arm arm;
    public activeIntake intake;

    private ElapsedTime timer;

    public RunAction scoreHigh;
    public universalAutomations(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
        intake = new activeIntake(hardwareMap);

        timer = new ElapsedTime();

        scoreHigh = new RunAction(this::scoreHigh);
    }

    public void scoreHigh() {
        // Start the timer
        timer.reset();

        // Sequence logic
        if (timer.seconds() < 1.0) {
            // Step 1: Raise arm to scoring position
            arm.max();
        } else if (timer.seconds() < 2.0) {
            // Step 2: Extend arm upwards
            arm.fullExtend();
        } else if (timer.seconds() < 3.0) {
            // Step 3: Position intake for scoring
            intake.pivotScore();
        }
    }
}
