package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.support.RunAction;

@Config
public class Diffy extends SubsystemBase {
    private Servo leftDiffyServo, rightDiffyServo;
    private ServoStates currentState;
    private final ElapsedTime switchTimer = new ElapsedTime();
    public static enum ServoStates {
        CENTER(0.50, 0.50),
        START(0.92, 0.885),
        END(0.2, 0.8),
        SPECIMEN(0.68,0.67),
        SUB(0.7, 0.3);


        private final double positionL, positionR;
        ServoStates(double positionL, double positionR) {
            this.positionL = positionL;
            this.positionR = positionR;
        }

        private double getPositionL() {
            return this.positionL;
        }
        private double getPositionR() {
            return this.positionR;
        }
    }

    // public RunAction centerDiffy, startDiffy, endDiffy, subDiffy, specimenDiffy;

    public Diffy(HardwareMap h) {
        leftDiffyServo = h.get(Servo.class, "leftDiffyServo");
        rightDiffyServo = h.get(Servo.class, "rightDiffyServo");
        currentState = ServoStates.CENTER;
        leftDiffyServo.setPosition(currentState.getPositionL());
        rightDiffyServo.setPosition(currentState.getPositionR());

        // switchTimer.reset();

        // centerDiffy = new RunAction(this::centerDiffy);
        // startDiffy = new RunAction(this::startDiffy);
        // endDiffy = new RunAction(this::endDiffy);
        // subDiffy = new RunAction(this::subDiffy);
        // specimenDiffy = new RunAction(this::specimenDiffy);
    }

    public void centerDiffy() {
        setPosition(ServoStates.CENTER);
    }
    public void specimenDiffy(){setPosition(ServoStates.SPECIMEN);}

    public void startDiffy() {
        setPosition(ServoStates.START);
    }

    public void endDiffy() {
        setPosition(ServoStates.END);
    }
    public void subDiffy() {
        setPosition(ServoStates.SUB);
    }

    //public static double testL = 0.500, testR = 0.500;
    public void manualRotate(double position) {
        leftDiffyServo.setPosition(position);
        rightDiffyServo.setPosition(position);
    }
    public void manualSpin(double offset) {
        leftDiffyServo.setPosition(leftPosition()-offset);
        rightDiffyServo.setPosition(rightPosition()+offset);
    }

    //TODO: TEST AND CHANGE BELOW
    public void moveDiffyP() {
        leftDiffyServo.setPosition(leftDiffyServo.getPosition() + 0.01);
        rightDiffyServo.setPosition(rightDiffyServo.getPosition() + 0.01);
    }

    public void moveDiffyN() {
        leftDiffyServo.setPosition(leftDiffyServo.getPosition() - 0.01);
        rightDiffyServo.setPosition(rightDiffyServo.getPosition() - 0.01);
    }

    public void rotateDiffyL() {
        leftDiffyServo.setPosition(leftDiffyServo.getPosition() + 0.01);
        rightDiffyServo.setPosition(rightDiffyServo.getPosition() - 0.01);
    }

    public void rotateDiffyR() {
        leftDiffyServo.setPosition(leftDiffyServo.getPosition() - 0.01);
        rightDiffyServo.setPosition(rightDiffyServo.getPosition() + 0.01);
    }

    public void setPosition(ServoStates state) {
        currentState = state;
        leftDiffyServo.setPosition(state.getPositionL());
        rightDiffyServo.setPosition(state.getPositionR());
    }

    public double leftPosition() {
        return leftDiffyServo.getPosition();
    }
    public double rightPosition() {
        return rightDiffyServo.getPosition();
    }
}
