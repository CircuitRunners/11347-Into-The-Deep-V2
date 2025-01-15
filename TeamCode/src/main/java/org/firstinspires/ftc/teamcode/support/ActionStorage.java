package org.firstinspires.ftc.teamcode.support;

public class ActionStorage {
    public org.firstinspires.ftc.teamcode.support.RunAction preparePurple = new org.firstinspires.ftc.teamcode.support.RunAction(this::preparePurple);
    public org.firstinspires.ftc.teamcode.support.RunAction scorePurple = new org.firstinspires.ftc.teamcode.support.RunAction(this::scorePurple);
    public org.firstinspires.ftc.teamcode.support.RunAction prepareYellow = new org.firstinspires.ftc.teamcode.support.RunAction(this::prepareYellow);
    public org.firstinspires.ftc.teamcode.support.RunAction scoreYellow = new org.firstinspires.ftc.teamcode.support.RunAction(this::scoreYellow);
    public org.firstinspires.ftc.teamcode.support.RunAction preparePark = new org.firstinspires.ftc.teamcode.support.RunAction(this::preparePark);
    public org.firstinspires.ftc.teamcode.support.RunAction finishPark = new RunAction(this::finishPark);
    //public SequentialAction auto = new SequentialAction(new ParallelAction(preparePurple, new SleepAction(1)), scorePurple, prepareYellow, scoreYellow, preparePark, finishPark);

    public void preparePurple() {}
    public void scorePurple() {}
    public void prepareYellow() {}
    public void scoreYellow() {}
    public void preparePark() {}
    public void finishPark() {}
}
