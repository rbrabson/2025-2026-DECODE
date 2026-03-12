package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Indexer;
import org.firstinspires.ftc.teamcode.hardware.IntakeMotor;

/**
 * Mechanism class for the Intake mechanism.
 */
public class Intake implements Mechanism, AutoCloseable {
    private final Indexer indexer;
    private final IntakeMotor intake;

    /**
     * Constructor for the Intake mechanism.
     *
     * @param hardwareMap The hardware map to initialize hardware devices.
     * @param telemetry   The telemetry object for logging.
     */
    public Intake(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.indexer = new Indexer(hardwareMap, telemetry);
        intake = new IntakeMotor(hardwareMap, telemetry);

        telemetry.addLine("Intake initialized");
    }

    /**
     * Starts the intake motor.
     */
    public void startIntakeMotor() {
        intake.start();
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntakeMotor() {
        intake.stop();
    }

    /**
     * Picks up an artifact and updates the current colors based on the intake
     * position.
     */
    public void pickupArtifacts() {
        indexer.intakeArtifact();
    }

    /**
     * Rotates the intake to the shooting position to transfer an artifact.
     *
     * @return the status of the action: SUCCESS if the artifact was
     *         successfully rotated to the shooting position, RUNNING if still
     *         in progress, FAILURE if the action failed.
     */
    public Status rotateToShootingPosition() {
        return indexer.shootArtifact();
    }

    /**
     * Checks if the intake is full, meaning it has an artifact ready
     *
     * @return true if the intake is full, false otherwise
     */
    public boolean isFull() {
        return indexer.isFull();
    }

    /**
     * Checks if the intake is empty, meaning it has no artifacts ready
     *
     * @return true if the intake is empty, false otherwise
     */
    public boolean isEmpty() {
        return indexer.isEmpty();
    }

    /**
     * Updates the indexer state to reflect that an artifact has been transferred from the
     * intake to the transfer mechanism.
     */
    public void artifactTransferred() {
        indexer.transferComplete();
    }

    @Override
    public void update() {
        // NO-OP
    }

    @Override
    public void close() {
        indexer.close();
    }
}
