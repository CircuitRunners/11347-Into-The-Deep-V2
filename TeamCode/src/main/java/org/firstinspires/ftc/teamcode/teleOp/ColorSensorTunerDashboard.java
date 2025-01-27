package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@TeleOp(name = "Color Sensor Tuner with Dashboard", group = "TeleOp")
public class ColorSensorTunerDashboard extends OpMode {

    private RevColorSensorV3 colorSensor;

    // Thresholds for color detection (accessible from FTC Dashboard)
    public static int redR = 255;
    public static int redG = 0;
    public static int redB = 0;

    public static int blueR = 0;
    public static int blueG = 0;
    public static int blueB = 255;

    public static int yellowR = 255;
    public static int yellowG = 255;
    public static int yellowB = 0;

    private int activeColor = 0; // 0 = Red, 1 = Blue, 2 = Yellow

    // Active color thresholds
    private int[] activeThreshold = {redR, redG, redB};

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addLine("Use FTC Dashboard to tune color thresholds.");
        telemetry.addLine("Press A to cycle active color (Red, Blue, Yellow).");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get current RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Update active color thresholds based on the selected color
        if (activeColor == 0) {
            activeThreshold[0] = redR;
            activeThreshold[1] = redG;
            activeThreshold[2] = redB;
        } else if (activeColor == 1) {
            activeThreshold[0] = blueR;
            activeThreshold[1] = blueG;
            activeThreshold[2] = blueB;
        } else if (activeColor == 2) {
            activeThreshold[0] = yellowR;
            activeThreshold[1] = yellowG;
            activeThreshold[2] = yellowB;
        }

        // Cycle through active color using the "A" button
        if (gamepad1.a) {
            activeColor = (activeColor + 1) % 3;
            sleep(200); // Prevent button spam
        }

        // Display color data and thresholds on telemetry
        telemetry.addData("Raw Sensor Data", "R: %d, G: %d, B: %d", red, green, blue);

        String activeColorName = activeColor == 0 ? "Red" :
                activeColor == 1 ? "Blue" : "Yellow";

        telemetry.addData("Active Color", activeColorName);
        telemetry.addData("Thresholds", "R: %d, G: %d, B: %d",
                activeThreshold[0], activeThreshold[1], activeThreshold[2]);

        // Determine the detected color
        if (isColorMatch(red, green, blue, new int[]{redR, redG, redB})) {
            telemetry.addLine("Detected Color: RED");
        } else if (isColorMatch(red, green, blue, new int[]{blueR, blueG, blueB})) {
            telemetry.addLine("Detected Color: BLUE");
        } else if (isColorMatch(red, green, blue, new int[]{yellowR, yellowG, yellowB})) {
            telemetry.addLine("Detected Color: YELLOW");
        } else {
            telemetry.addLine("Detected Color: NONE");
        }

        telemetry.update();
    }

    /**
     * Checks if the detected RGB values match the given thresholds.
     */
    private boolean isColorMatch(int r, int g, int b, int[] threshold) {
        return Math.abs(r - threshold[0]) < 20 &&
                Math.abs(g - threshold[1]) < 20 &&
                Math.abs(b - threshold[2]) < 20;
    }

    /**
     * Simple delay method for debouncing buttons.
     */
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
