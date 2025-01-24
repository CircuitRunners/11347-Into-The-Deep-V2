package org.firstinspires.ftc.teamcode.subsystems.Testing;

import com.arcrobotics.ftclib.command.CommandBase;

public class ArmRetPID extends CommandBase {
    private TwoBitMechanisms mechanisms;
    private double target;

    public ArmRetPID(TwoBitMechanisms mech, double target) {
        mechanisms = mech;
        this.target = target;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        mechanisms.retPID(target);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mechanisms.getPosition() - target) < 300;
    }

    @Override
    public void end(boolean interrupted) {
        mechanisms.runManual(0, 0);
    }
}
