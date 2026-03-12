package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * The Flywheel class represents the flywheel mechanism of the robot, which is
 * responsible for launching game elements. It provides methods to initialize
 * the flywheel motor, set its velocity, and retrieve its current velocity.
 */
public class Flywheel {
    public static final double VELOCITY_HIGH = 1500;
    public static final double VELOCITY_LOW = 1170;
    public static final double VELOCITY_CLOSE = 500;
    public static final double VELOCITY_FAR = 1120;
    public static final double OFF_VELOCITY = 0;

    private static final double ACCEPTABLE_VELOCITY_ERROR = 50;

    private static final double P = 160;
    private static final double I = 0;
    private static final double D = 0;
    private static final double F = 15;

    private static final double TELEMETRY_LOG_DELTA = 10.0;
    private static final double TELEMETRY_LOG_INTERVAL_MS = 1000;

    private final DcMotorEx flywheel;
    @Nullable private final Telemetry telemetry;

    private double targetVelocity = 0;

    private long lastTelemetryLogMs = 0;
    private double lastLoggedVelocity = Double.NaN;

    /**
     * Constructor for the Flywheel class, which initializes the flywheel motor used by the shooter.
     *
     * @param hardwareMap the hardware map to access the servo
     */
    public Flywheel(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for the Flywheel class, which initializes the flywheel motor used by the shooter.
     *
     * @param hardwareMap the hardware map to access the servo
     * @param telemetry   the telemetry object for logging information
     */
    public Flywheel(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.flywheel = map.get(DcMotorEx.class, "shoot1");
        this.telemetry = telemetry;

        initializeFlywheelMotor();

        if (telemetry != null) {
            telemetry.addLine("Flywheel initialized");
        }
    }

    /**
     * Initializes the flywheel motor with the appropriate direction, PIDF
     * coefficients, and initial velocity.
     */
    private void initializeFlywheelMotor() {
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        flywheel.setVelocity(0);
    }

    /**
     * Sets the flywheel motor to a high velocity, which is defined as a constant
     * value (VELOCITY_HIGH) that can be adjusted based on the desired performance
     * of the flywheel mechanism. This method allows for quick and easy adjustments
     * to the flywheel's speed without needing to specify the velocity value each time.
     */
    public void setHigh() {
        setVelocity(VELOCITY_HIGH);
    }

    /**
     * Sets the flywheel motor to a low velocity, which is defined as a constant
     * value (VELOCITY_LOW) that can be adjusted based on the desired performance of
     * the flywheel mechanism. This method allows for quick and easy adjustments to
     * the flywheel's speed without needing to specify the velocity value each time.
     */
    public void setLow() {
        setVelocity(VELOCITY_LOW);
    }

    /**
     * Sets the flywheel motor to a value used for close shots during autonomous
     * mode. The velocity is defined as a constant value (VELOCITY_CLOSE) that can
     * be adjusted based on the desired performance of the flywheel mechanism. This
     * method allows for quick and easy adjustments to the flywheel's speed for
     * close shots without needing to specify the velocity value each time.
     */
    public void setAutonomousClose() {
        setVelocity(VELOCITY_CLOSE);
    }

    /**
     * Sets the flywheel motor to a value used for far shots during autonomous mode.
     * The velocity is defined as a constant value (VELOCITY_FAR) that can be
     * adjusted based on the desired performance of the flywheel mechanism. This
     * method allows for quick and easy adjustments to the flywheel's speed for far
     * shots without needing to specify the velocity value each time.
     */
    public void setAutonomousFar() {
        setVelocity(VELOCITY_FAR);
    }

    /**
     * Turns off the flywheel motor by setting its velocity to zero, which is
     * defined as a constant value (OFF_VELOCITY). This method allows for quick and
     * easy stopping of the flywheel mechanism without needing to specify the
     * velocity value each time.
     */
    public void turnOff() {
        setVelocity(OFF_VELOCITY);
    }

    /**
     * Checks if the flywheel motor is currently at the target velocity by comparing
     * the current velocity (obtained using the getVelocity method) to the target
     * velocity (stored in the targetVelocity variable). If the absolute difference
     * between the current velocity and the target velocity is less than or equal to
     * a defined acceptable velocity error (ACCEPTABLE_VELOCITY_ERROR), this method
     * returns true, indicating that the flywheel is at the target velocity. Otherwise,
     * it returns false, indicating that the flywheel is not yet at the target velocity.
     *
     * @return true if the flywheel is at the target velocity, false otherwise.
     */
    public boolean isAtTargetVelocity() {
        return Math.abs(getVelocity() - targetVelocity) <= ACCEPTABLE_VELOCITY_ERROR;
    }

    /**
     * Returns the current velocity of the flywheel motor.
     *
     * @return The current velocity of the flywheel motor, obtained using the
     *         getVelocity method of the DcMotorEx class.
     */
    public double getVelocity() {
        return flywheel.getVelocity();
    }

    /**
     * Sets the velocity of the flywheel motor to a specified value.
     *
     * @param velocity The desired velocity for the flywheel motor, which will be
     *                 set using the setVelocity method of the DcMotorEx class.
     */
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        flywheel.setVelocity(velocity);

        if (telemetry != null) {
            long now = System.currentTimeMillis();
            boolean intervalElapsed = (now - lastTelemetryLogMs) >= TELEMETRY_LOG_INTERVAL_MS;
            boolean changedEnough = Double.isNaN(lastLoggedVelocity)
                    || Math.abs(velocity - lastLoggedVelocity) >= TELEMETRY_LOG_DELTA;

            if (intervalElapsed || changedEnough) {
                telemetry.addData("Flywheel Velocity", velocity);
                lastTelemetryLogMs = now;
                lastLoggedVelocity = velocity;
            }
        }
    }
}
