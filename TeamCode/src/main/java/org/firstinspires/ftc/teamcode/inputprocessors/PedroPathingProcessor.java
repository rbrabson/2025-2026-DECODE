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
    private final PedroPathingDrive drive;
    @Nullable private final Telemetry telemetry;

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
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        drive.setTeleOpDrive(forward, strafe, turn, false);

        if (telemetry != null) {
            Pose pose = drive.getPose();
            telemetry.addData("Drive X", pose.getX());
            telemetry.addData("Drive Y", pose.getY());
            telemetry.addData("Drive Heading", Math.toDegrees(pose.getHeading()));
        }
    }
}
