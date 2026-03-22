package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Class representing the hood mechanism of the robot, which is responsible for
 * adjusting the angle of the shooter. The hood position can be increased or
 * decreased within defined limits.
 */
public class Hood {
    private static final double HOOD_POSITION_MINIMUM = 0.0;
    private static final double HOOD_POSITION_MAXIMUM = 1.0;

    private static final double POSITION_EPSILON = 1e-4;

    private final Servo hood;
    private final Telemetry telemetry;

    /**
     * Constructor for the Hood class, which initializes the hood servo and telemetry.
     *
     * @param hardwareMap The hardware map to access the hood servo.
     * @param telemetry   The telemetry object for logging data related to the hood
     *                    mechanism.
     */
    public Hood(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.hood = map.get(Servo.class, "s1");
        this.telemetry = telemetry;

        telemetry.addLine("Hood initialized");
    }

    /**
     * Sets the hood to its starting position, which is defined by HOOD_POSITION_START.
     */
    public void retract() {
        setPosition(HOOD_POSITION_MINIMUM);
    }

    /**
     * Sets the hood to its maximum position, which is defined by HOOD_POSITION_MAXIMUM.
     */
    public void fullyExtend() {
        setPosition(HOOD_POSITION_MAXIMUM);
    }

    /**
     * Decreases the hood position by a specified increment, ensuring that the new
     * position does not go below the defined minimum.
     *
     * @param increment The amount to decrease the hood position by.
     */
    public void decreasePosition(double increment) {
        setPosition(getPosition() - increment);
    }

    /**
     * Returns the current position of the hood servo.
     *
     * @return The current position of the hood servo.
     */
    public double getPosition() {
        return hood.getPosition();
    }

    /**
     * Sets the hood position to a specified value, ensuring that it is within the
     * defined minimum and maximum limits.
     *
     * @param position The desired position for the hood, which will be clipped to
     *                 the range defined by HOOD_POSITION_MINIMUM and
     *                 HOOD_POSITION_MAXIMUM.
     */
    public void setPosition(double position) {
        double clippedPosition = Range.clip(position, HOOD_POSITION_MINIMUM, HOOD_POSITION_MAXIMUM);
        if (Math.abs(clippedPosition - hood.getPosition()) < POSITION_EPSILON) {
            return;
        }
        hood.setPosition(clippedPosition);
        telemetry.addData("[HOOD] Position", clippedPosition);
    }

    /**
     * Increases the hood position by a specified increment, ensuring that the new
     * position does not exceed the defined maximum.
     *
     * @param increment The amount to increase the hood position by.
     */
    public void increasePosition(double increment) {
        setPosition(getPosition() + increment);
    }
}
