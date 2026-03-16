package org.firstinspires.ftc.teamcode.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.Motif;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;

/**
 * Class responsible for interfacing with the Limelight camera to detect motifs
 * and calculate aiming errors.
 */
public class Limelight {
    public static final int PIPELINE_MOTIF = 0;
    public static final int PIPELINE_BLUE_GOAL = 1;
    public static final int PIPELINE_RED_GOAL = 2;

    private final Limelight3A limelight;
    @Nullable private final Telemetry telemetry;

    private Motif detectedMotif = null; // Default to GPP

    /**
     * Constructor for the Limelight class. Initializes the Limelight camera without any telemetry.
     *
     * @param hardwareMap The hardware map to access the Limelight camera.
     */
    public Limelight(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for the Limelight class. Initializes the Limelight camera and
     * sets up the telemetry.
     *
     * @param hardwareMap The hardware map to access the Limelight camera.
     * @param telemetry   The telemetry object to send data back to the driver
     *                    station.
     */
    public Limelight(@NotNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;

        initializeLimelight();

        if (telemetry != null) {
            telemetry.addLine("Limelight initialized");
        }
    }

    /**
     * Initializes the Limelight camera by setting the transmission interval,
     * switching to the motif detection pipeline and starting the camera.
     */
    private void initializeLimelight() {
        switchToDetectMotif();
        limelight.start();
    }

    /**
     * Switches the Limelight camera to the motif detection pipeline. This method can be called
     * to reset the camera to detect motifs after it has been switched to a different pipeline for
     * aiming or shooting.
     */
    public void switchToDetectMotif() {
        limelight.pipelineSwitch(PIPELINE_MOTIF);
        if (telemetry != null) {
            telemetry.addData("Pipeline", "detect motif");
        }
    }

    /**
     * Detects the motif of the game element by analyzing the latest results from
     * the Limelight camera. If a valid result is found, it extracts the AprilTag ID
     * and maps it to the corresponding motif. If no valid result is found, it defaults to GPP.
     *
     * @return The detected motif, or GPP if no valid result is found.
     */
    public Motif detectMotif() {
        if (detectedMotif != null) {
            if (telemetry != null) {
                telemetry.addData("[LIMELIGHT] Motif", detectedMotif.name());
            }
            return detectedMotif; // Return previously detected motif if already detected
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            if (telemetry != null) {
                telemetry.addData("[LIMELIGHT] Motif", "No valid result");
            }
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            if (telemetry != null) {
                telemetry.addData("[LIMELIGHT] Motif", "No fiducials detected");
            }
            return null; // Default to GPP if no fiducials detected
        }

        int tagId = fiducials.get(0).getFiducialId();
        detectedMotif = Motif.fromAprilTagID(tagId);
        if (telemetry != null) {
            telemetry.addData("[LIMELIGHT] Motif", detectedMotif.name());
        }
        return detectedMotif;
    }

    /**
     * Switches the Limelight camera to the shooting pipeline for aiming at the blue goal. This
     * method can be called after detecting the motif to prepare the camera for aiming at the
     * target. The specific pipeline used for aiming at the blue goal is defined by the
     * PIPELINE_BLUE_GOAL constant.
     */
    public void switchToBlueGoal() {
        limelight.pipelineSwitch(PIPELINE_BLUE_GOAL);
        if (telemetry != null) {
            telemetry.addData("[LIMELIGHT] Pipeline", "blue goal");
        }
    }

    /**
     * Switches the Limelight camera to the shooting pipeline for aiming at the red goal. This
     * method can be called after detecting the motif to prepare the camera for aiming at the
     * target. The specific pipeline used for aiming at the red goal is defined by the
     * PIPELINE_RED_GOAL constant.
     */
    public void switchToRedGoal() {
        limelight.pipelineSwitch(PIPELINE_RED_GOAL);
        if (telemetry != null) {
            telemetry.addData("[LIMELIGHT] Pipeline", "red goal");
        }
    }

    /**
     * Checks if a motif has been detected by the Limelight camera. This method
     * returns true if a valid motif has been detected and stored, and false
     * otherwise.
     *
     * @return True if a motif has been detected, false otherwise.
     */
    public boolean isMotifDetected() {
        return detectedMotif != null;
    }

    /**
     * Calculates the horizontal angle error (tx) from the latest results of the
     * Limelight camera. If a valid result is found, it extracts the tx value and
     * returns it. If no valid result is found, it returns NaN to indicate that the
     * error cannot be calculated.
     *
     * @return The horizontal angle error (tx) if a valid result is found, or NaN if
     *         no valid result is available.
     */
    public double getError() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            if (telemetry != null) {
                telemetry.addData("[LIMELIGHT] Error", "no valid result");
            }
            return Double.NaN; // Return NaN if no valid result
        }

        if (telemetry != null) {
            telemetry.addData("[LIMELIGHT] Error", result.getTx());
        }
        return result.getTx();
    }

    /**
     * Gets the latest result from the Limelight camera. This method can be used to access the raw
     * data from the camera for more advanced processing or debugging purposes
     *
     * @return The latest LLResult from the Limelight camera, or null if no valid result is available.
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Gets the Limelight3A sensor object. This method can be used to access the underlying
     * Limelight3A object for more advanced operations or configurations that are not exposed by
     * the Limelight class.
     *
     * @return The Limelight3A sensor object used to interface with the Limelight camera.
     */
    public Limelight3A getSensor() {
        return limelight;
    }
}
