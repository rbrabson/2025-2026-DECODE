package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Drive;

import java.util.Objects;

/**
 * A controller for a mecanum drive chassis. This controller uses the left stick on gamepad1 to
 * control the X and Y movement of the robot, and the right stick on gamepad1 to control the
 * turning of the robot. The left bumper on gamepad1 can be used to toggle the gain for the X
 * axis, Y axis, and turn speed.
 */
public class DriveProcessor implements UserInputProcessor {
    private static final double TURN_MULTIPLIER = 0.75; // Max turning speed multiplier
    private static final double NORMAL_GAIN = 1.0;
    private static final double SLOW_GAIN = 0.5;

    private final Drive drive;
    @Nullable private final Telemetry telemetry;

    private boolean slowMode = false;

    /**
     * Creates a controller for a mecanum drive chassis.
     *
     * @param drive     the drive to control.
     * @param telemetry the telemetry used to display data on the driver station.
     */
    public DriveProcessor(@NonNull Drive drive, @Nullable Telemetry telemetry) {
        this.drive = Objects.requireNonNull(drive);
        this.telemetry = telemetry;
    }

    /**
     * Updates the robot drive speed based on the left and right joysticks on gamepad1.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    @Override
    public void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        double gain = slowMode ? SLOW_GAIN : NORMAL_GAIN;

        double x = -gamepad1.left_stick_x * gain;
        double y = -gamepad1.left_stick_y * gain;
        double turn = -gamepad1.right_stick_x * gain * TURN_MULTIPLIER;

        drive.drive(x, y, turn);

        if (telemetry != null) {
            telemetry.addData("[DRIVE] X", x);
            telemetry.addData("[DRIVE] Y", y);
            telemetry.addData("[DRIVE] Turn", turn);
            telemetry.addData("[DRIVE] Slow Mode", slowMode);
        }
    }

    /**
     * Gets a string representation of this mecanum drive controller.
     *
     * @return a string representation of this mecanum drive controller
     */
    @NonNull
    @Override
    public String toString() {
        return "DriveProcessor{" +
                "Drive=" + drive +
                '}';
    }
}
