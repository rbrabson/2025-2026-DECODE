package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import java.util.Objects;

/**
 * The IntakeInput class manages the control of the intake mechanism based on gamepad inputs.
 */
public class IntakeProcessor implements UserInputProcessor {
    private final static long INTAKE_TOGGLE_DELAY = 500;
    private final Intake intake;
    @Nullable private final Telemetry telemetry;

    private final Timer rightTriggerDuration = new Timer();
    private boolean isIntakeRunning = true;
    private boolean rightTriggerLatched = false;

    /**
     * Constructor for the IntakeInput class.
     * @param intake    the Intake mechanism to control
     * @param telemetry the Telemetry for debugging and feedback
     */
    public IntakeProcessor(@NonNull Intake intake, @Nullable Telemetry telemetry) {
        this.intake = Objects.requireNonNull(intake);
        this.telemetry = telemetry;
    }

    /**
     * Process the gamepad inputs to control the intake mechanism.
     *
     * @param gamepad1 The current state of gamepad1.
     * @param gamepad2 The current state of gamepad2.
     */
    @Override
    public void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        boolean oldIsIntakeRunning = isIntakeRunning;
        boolean triggerPressed = gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5;
        boolean toggleIntakeMotor = triggerPressed && !rightTriggerLatched && rightTriggerDuration.getElapsedTime() > INTAKE_TOGGLE_DELAY;
        if (toggleIntakeMotor) {
            isIntakeRunning = !isIntakeRunning;
            rightTriggerDuration.resetTimer();
        }
        rightTriggerLatched = triggerPressed;

        if (isIntakeRunning) {
            intake.startIntakeMotor();
        } else {
            intake.stopIntakeMotor();
        }

        if (telemetry != null && isIntakeRunning != oldIsIntakeRunning) {
            telemetry.addData("[INTAKE] Running", isIntakeRunning);
        }
    }
}
