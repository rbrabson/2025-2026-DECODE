package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Class representing the hinge mechanism of the robot. This is used to transfer an artifact
 * from the intake to the shooter.
 */
public class Hinge {
    private static final double RAISED_POSITION = 0.4;
    private static final double LOWERED_POSITION = 0.09;

    private static final double HINGE_DELAY_MS = 175;
    private static final double POSITION_EPSILON =1e-4;

    private final Servo hinge;
    private final Telemetry telemetry;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor for the Hinge class.
     *
     * @param hardwareMap the hardware map to access the servo
     * @param telemetry   the telemetry object for logging information
     */
    public Hinge(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.hinge = map.get(Servo.class, "h");
        this.telemetry =Objects.requireNonNull(telemetry);

        telemetry.addLine("Hinge initialized");
    }

    /**
     * Moves the hinge to the raised position.
     */
    public void raise() {
        setPosition(RAISED_POSITION);
        telemetry.addLine("[HINGE] Raised");
    }

    /**
     * Sets the position of the hinge servo.
     *
     * @param position the target position for the hinge servo (between 0.0 and 1.0)
     */
    private void setPosition(double position) {
        if (position == hinge.getPosition()) {
            return;
        }
        hinge.setPosition(position);
        timer.reset();
        telemetry.addData("[HINGE] Position", position);
    }

    /**
     * Moves the hinge to the lowered position.
     */
    public void lower() {
        setPosition(LOWERED_POSITION);
        telemetry.addLine("[HINGE] Lowered");
    }

    /**
     * Checks if the hinge is in the raised position and has completed its movement.
     *
     * @return true if the hinge is raised and ready for the next action, false otherwise
     */
    public boolean isRaised() {
        return Math.abs(hinge.getPosition() - RAISED_POSITION) < POSITION_EPSILON && isAtTarget();
    }

    /**
     * Checks if the hinge has completed its movement and is ready for the next action.
     *
     * @return true if the hinge is ready, false otherwise
     */
    public boolean isAtTarget() {
        return timer.milliseconds() >= HINGE_DELAY_MS;
    }

    public boolean isLowered() {
        return Math.abs(hinge.getPosition() - LOWERED_POSITION) < POSITION_EPSILON && isAtTarget();
    }
}
