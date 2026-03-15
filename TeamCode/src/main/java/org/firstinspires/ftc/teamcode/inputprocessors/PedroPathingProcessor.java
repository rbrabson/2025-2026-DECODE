package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.PedroPathingDrive;

/**
 * Example input processor for a pathing drive system. This processor allows the driver to press
 * buttons to automatically drive to predefined poses on the field, while still allowing manual
 * control when not auto-driving.
 */
public class PedroPathingProcessor implements UserInputProcessor {
    private static final double NORMAL_GAIN = 1.0;
    private static final double SLOW_GAIN = 0.5;

    private final PedroPathingDrive drive;
    @Nullable private final Telemetry telemetry;

    private boolean slowMode = false;

    /**
     * Constructor for the PedroPathingProcessor with only the drive parameter. Telemetry will be
     * set to null.
     *
     * @param drive The PedroPathingDrive instance used to control the robot's movement.
     */
    public PedroPathingProcessor(@NonNull PedroPathingDrive drive) {
        this(drive, null);
    }

    /**
     * Constructor for the PedroPathingProcessor.
     *
     * @param drive     The PedroPathingDrive instance used to control the robot's movement.
     * @param telemetry The Telemetry instance used for debugging and feedback during operation.
     */
    public PedroPathingProcessor(@NonNull PedroPathingDrive drive, @NonNull Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
    }

    /**
     * Processes the gamepad input to control the robot. Pressing specific buttons will trigger auto-driving
     * to predefined poses, while manual control is available when not auto-driving.
     *
     * @param gamepad1 The current state of gamepad1.
     * @param gamepad2 The current state of gamepad2.
     */
    @Override
    public void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }
        double gain = slowMode ? SLOW_GAIN : NORMAL_GAIN;

        double forward = -gamepad1.left_stick_y * gain;
        double strafe = -gamepad1.left_stick_x * gain;
        double turn = -gamepad1.right_stick_x * gain;

        drive.drive(forward, strafe, turn);

        if (telemetry != null) {
            Pose pose = drive.getPose();
            telemetry.addData("Drive X", pose.getX());
            telemetry.addData("Drive Y", pose.getY());
            telemetry.addData("Drive Heading", Math.toDegrees(pose.getHeading()));
        }
    }
}
