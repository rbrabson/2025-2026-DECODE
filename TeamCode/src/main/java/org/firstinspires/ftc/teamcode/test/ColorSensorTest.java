package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/**
 * Tests the color sensor (webcam-based) for ball detection.
 *
 * This test initializes the webcam color processor and continuously
 * displays the detected color. Use this to:
 * - Verify the webcam "logi" is connected
 * - Check that green and purple balls are correctly identified
 * - Adjust the ROI (Region of Interest) if needed
 * - See what colors are detected when no ball is present
 *
 * CONTROLS:
 *   (No gamepad controls - display only)
 *
 * Place a green or purple ball in front of the intake and watch
 * the detected color update in real-time.
 *
 * TODO: [Student] What color does the sensor report when no ball is present?
 *       What about when the intake slot is between two balls?
 *       How could you improve detection reliability?
 */
public class ColorSensorTest implements BaseTest {

    private PredominantColorProcessor colorSensor;
    private VisionPortal portal;

    @Override
    public void init(HardwareMap hardwareMap) {
        colorSensor = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, -0.5, 0.4, -0.8))
            .setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.WHITE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLUE
            )
            .build();

        portal = new VisionPortal.Builder()
            .addProcessor(colorSensor)
            .setCameraResolution(new Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName.class, "logi"))
            .build();
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();

        telemetry.addLine("=== Color Sensor Test ===");
        telemetry.addLine("Place a ball in front of the intake");
        telemetry.addLine("");
        telemetry.addData("Closest Match", result.closestSwatch);
        telemetry.addLine("");

        // Show interpretation
        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
            telemetry.addLine(">>> GREEN BALL DETECTED <<<");
        } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
            telemetry.addLine(">>> PURPLE BALL DETECTED <<<");
        } else {
            telemetry.addLine("No green/purple ball detected");
            telemetry.addData("Seeing", result.closestSwatch);
        }

        telemetry.addLine("");
        telemetry.addLine("ROI: (0.2, -0.5) to (0.4, -0.8)");
    }

    @Override
    public void stop() {
        if (portal != null) {
            portal.close();
        }
    }
}
