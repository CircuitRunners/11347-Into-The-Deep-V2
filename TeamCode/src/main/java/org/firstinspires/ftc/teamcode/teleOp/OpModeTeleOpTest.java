//package org.firstinspires.ftc.teamcode.teleOp;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.ArmRot;
//
//import java.util.List;
//
//@Config
//@TeleOp
//public class OpModeTeleOpTest extends OpMode {
//    private ArmRot arm;
//    private Telemetry telem = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//    public int test;
//
//    public int retractionTarget = 0; // Remove this if not needed
//    public int rotationTarget = 300;
//
//    GamepadEx driver, manipulator;
//
//    @Override
//    public void init() {
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
////        arm = new Arm(hardwareMap);
//        arm = new ArmRot(hardwareMap, telem);
//
//        telem.addLine("ready");
//        telem.update();
//    }
//
//    public void armTest() {
//        switch (test) {
//            case 1:
//                arm.setTarget(1000);
//                break;
//            case 2:
//                arm.setTarget(5000);
//                break;
//        }
//    }
//
//    public void setTestState(int x) {
//        test = x;
//    }
//
//    @Override
//    public void loop() {
//        arm.update();
//
//        if (gamepad1.a) {
//            setTestState(1);
//        }
//        if (gamepad1.b) {
//            setTestState(2);
//        }
//
//        armTest();
//
//        telem.addData("Arm Pos", arm.getCurrentRotation());
//        telem.addData("Arm Target", arm.getTarget());
//        telem.update();
//    }
//
//    @Override
//    public void start() {
//        setTestState(1);
//    }
//}
