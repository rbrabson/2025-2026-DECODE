package org.firstinspires.ftc.teamcode.sensors;

import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.decode.Color;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/**
 * Class responsible for detecting the color of the Artifacts using a webcam and
 * OpenCV.
 */
public class ColorProcessor implements AutoCloseable {
    @Nullable private final Telemetry telemetry;
    private PredominantColorProcessor colorSensor;
    @Nullable private VisionPortal visionPortal;

    /**
     * Constructor for the ColorSensor class. Initializes the webcam and sets up the
     * color detection pipeline.
     *
     * @param hardwareMap The hardware map to access the webcam.
     */
    public ColorProcessor(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for the ColorSensor class. Initializes the webcam and sets up the
     * color detection pipeline.
     *
     * @param hardwareMap The hardware map to access the webcam.
     * @param telemetry   The telemetry object to send data back to the driver
     *                    station.
     */
    public ColorProcessor(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "logi");
        this.telemetry = telemetry;
        initializeColorProcessor(webcam);

        if (telemetry != null) {
            telemetry.addLine("Color Sensor initialized");
        }
    }

    /**
     * Initializes the color processor by setting up the PredominantColorProcessor and
     * VisionPortal.
     *
     * @param webcam The webcam to use for color detection.
     */
    private void initializeColorProcessor(@NonNull WebcamName webcam) {
        this.colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, -0.5, 0.4, -0.8))
                .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, PredominantColorProcessor.Swatch.WHITE,
                        PredominantColorProcessor.Swatch.BLACK, PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLUE)
                .build();

        visionPortal = new VisionPortal.Builder().addProcessor(colorSensor).setCameraResolution(new Size(320, 240)).setCamera(webcam)
                .build();

    }

    /**
     * Detects the color of the Artifact by analyzing the webcam feed using the
     * PredominantColorProcessor.
     *
     * @return The detected color as an enum value (GREEN, PURPLE, or UNKNOWN).
     */
    @Nullable
    public Color detectColor() {
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();
        if (result == null) {
            if (telemetry != null) {
                telemetry.addLine("No color detected");
                telemetry.addData("[COLOR] Detected", "NONE");
            }
            return null;
        }

        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
            if (telemetry != null) {
                telemetry.addData("[COLOR] Detected", "GREEN");
            }
            return Color.GREEN;
        }

        if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
            if (telemetry != null) {
                telemetry.addData("[COLOR] Detected", "PURPLE");
            }
            return Color.PURPLE;
        }

        if (telemetry != null) {
            telemetry.addData("[COLOR] Detected", "UNKNOWN");
        }

        return Color.UNKNOWN;
    }

    /**
     * Closes the VisionPortal and releases any resources used by the color processor.
     */
    @Override
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
}
