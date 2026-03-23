package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

import java.util.Objects;

/**
 * The ShootArtifactProcessor class manages the interaction between multiple mechanisms to manage the flow
 * of artifacts from the intake, through the transfer mechanism, and into the shooter for scoring.
 */
public class ShootArtifactProcessor implements UserInputProcessor {
    private final Intake intake;
    private final Transfer transfer;
    private final Shooter shooter;
    private final Telemetry telemetry;

    // State to manage the FSM for shooting an artifact
    private enum FlowState {
        INACTIVE,
        ROTATING_INTAKE,
        WAITING_FOR_FLYWHEEL,
        TRANSFERRING,
        TRANSFER_COMPLETE }

    private FlowState flowState = FlowState.INACTIVE;
    private boolean leftTriggerLatched = false;

    /**
     * Creates a new ShootArtifactInput controller.
     * @param intake    the intake mechanism to control.
     * @param transfer  the transfer mechanism to control.
     * @param shooter   the shooter mechanism to control.
     * @param telemetry the telemetry used to display data on the driver station.
     */
    public ShootArtifactProcessor(@NonNull Intake intake, @NonNull Transfer transfer, @NonNull Shooter shooter, @NonNull Telemetry telemetry) {
        this.intake = Objects.requireNonNull(intake);
        this.transfer = Objects.requireNonNull(transfer);
        this.shooter = Objects.requireNonNull(shooter);
        this.telemetry = Objects.requireNonNull(telemetry);
    }

    /**
     * Processes the input from gamepad1 and gamepad2 to manage the flow of artifacts from the
     * intake, through the transfer mechanism, and into the shooter for scoring.
     *
     * @param gamepad1 The current state of gamepad1.
     * @param gamepad2 The current state of gamepad2.
     */
    @Override public void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        boolean triggerPressed = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        boolean transferRequested = triggerPressed && !leftTriggerLatched && !intake.isEmpty();

        switch (flowState) {
            case INACTIVE:
                if (transferRequested) {
                    flowState = FlowState.ROTATING_INTAKE;
                }
                break;

            case ROTATING_INTAKE:
                Status rotateStatus = intake.rotateToShootingPosition();
                if (rotateStatus == Status.SUCCESS) {
                    flowState = FlowState.WAITING_FOR_FLYWHEEL;
                } else if (rotateStatus == Status.FAILURE) {
                    flowState = FlowState.INACTIVE;
                }
                break;

            case WAITING_FOR_FLYWHEEL:
                if (shooter.isFlywheelAtTargetRPM()) {
                    flowState = FlowState.TRANSFERRING;
                }
                break;

            case TRANSFERRING:
                Status transferStatus = transfer.transferArtifact();
                if (transferStatus == Status.SUCCESS) {
                    flowState = FlowState.TRANSFER_COMPLETE;
                } else if (transferStatus == Status.FAILURE) {
                    flowState = FlowState.INACTIVE;
                }
                break;

            case TRANSFER_COMPLETE:
                intake.artifactTransferred();
                flowState = FlowState.INACTIVE;
                break;
        }

        leftTriggerLatched = triggerPressed;

        telemetry.addData("[SHOOT] State", flowState);
        telemetry.addData("[SHOOT] Trigger Latched", leftTriggerLatched);
    }
}
