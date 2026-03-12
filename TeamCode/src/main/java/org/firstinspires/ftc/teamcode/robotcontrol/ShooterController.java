package org.firstinspires.ftc.teamcode.robotcontrol;

import com.qualcomm.robotcore.util.Range;

/**
 * ShooterController uses cubic regression coefficients to predict flywheel velocity and hood position * based on distance. It also includes a low-pass filter for distance measurements and an * automatic shot tuning mechanism.
 */
public class ShooterController {
    // Distance filter
    private static final double FILTER_ALPHA = 0.2;

    // Distance domain used for regression validity
    private static final double MIN_SHOT_DISTANCE = 0.0;
    private static final double MAX_SHOT_DISTANCE = 200.0;

    // Flywheel cubic coefficients
    private static final double FA = -0.001660;
    private static final double FB = 0.297488;
    private static final double FC = 5.153846;
    private static final double FD = -930.461538;

    // Hood cubic coefficients
    private static final double HA = 0.000000;
    private static final double HB = 0.000000;
    private static final double HC = 0.000000;
    private static final double HD = -0.000000;

    // Output limits
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 2000.0;
    private static final double MIN_HOOD = 0.0;
    private static final double MAX_HOOD = 0.8;

    // Automatic shot tuning
    private static final double HOOD_OFFSET_STEP = 0.01;
    private static final double VELOCITY_OFFSET_STEP = 25.0;
    private static final double MIN_VELOCITY_OFFSET = -300.0;
    private static final double MAX_VELOCITY_OFFSET = 300.0;
    private static final double MIN_HOOD_OFFSET = -0.20;
    private static final double MAX_HOOD_OFFSET = 0.20;

    // Artifact flight constant (seconds per inch)
    private static final double FLIGHT_TIME_PER_INCH = 0.002;

    // Flywheel velocity correction factor for robot motion compensation
    public static final double FLYWHEEL_CORRECTION = 5.0;

    private double velocityOffset = 0.0;
    private double hoodOffset = 0.0;
    private double filteredDistance = 0.0;
    private boolean distanceFilterInitialized = false;

    /**
     * Update the filtered distance measurement using a low-pass filter.
     *
     * @param measuredDistance The latest distance measurement (in inches).
     * @return The updated filtered distance value.
     */
    public double updateDistance(double measuredDistance) {
        double clamped = Range.clip(measuredDistance, MIN_SHOT_DISTANCE, MAX_SHOT_DISTANCE);

        if (!distanceFilterInitialized) {
            filteredDistance = clamped;
            distanceFilterInitialized = true;
        } else {
            filteredDistance = (1.0 - FILTER_ALPHA) * filteredDistance + FILTER_ALPHA * clamped;
        }

        return filteredDistance;
    }

    /**
     * Calculate flywheel velocity for the given distance and robot closing velocity.
     *
     * @param distance The distance to the target (in inches).
     * @param robotVelocityTowardTarget Robot velocity toward target (in/s), positive when moving toward.
     * @return Flywheel setpoint in raw motor velocity units, clamped to limits.
     */
    public double getFlywheelVelocity(double distance, double robotVelocityTowardTarget) {
        double d = Range.clip(distance, MIN_SHOT_DISTANCE, MAX_SHOT_DISTANCE);
        double velocity = ((FA * d + FB) * d + FC) * d + FD;

        // Moving toward target reduces required launch speed.
        velocity -= robotVelocityTowardTarget * FLYWHEEL_CORRECTION;

        velocity += velocityOffset;
        return Range.clip(velocity, MIN_VELOCITY, MAX_VELOCITY);
    }

    /**
     * Calculate hood position for the given distance.
     *
     * @param distance The distance to the target (in inches).
     * @return Hood position (0 to1), clamped to limits.
     */
    public double getHoodPosition(double distance) {
        double d = Range.clip(distance, MIN_SHOT_DISTANCE, MAX_SHOT_DISTANCE);
        double hood = ((HA * d + HB) * d + HC) * d + HD;

        hood += hoodOffset;
        return Range.clip(hood, MIN_HOOD, MAX_HOOD);
    }

    /**
     * Calculate turret lead angle for lateral robot motion.
     *
     * @param distance The distance to target (in inches).
     * @param robotLateralVelocity Robot lateral velocity (in/s).
     * @param turretDistanceFromCenter Turret offset from robot center (in inches).
     * @return Lead angle in radians.
     */
    public double getTurretLeadAngle(double distance, double robotLateralVelocity, double turretDistanceFromCenter) {
        double d = Math.max(1e-6, distance);
        double flightTime = d * FLIGHT_TIME_PER_INCH;
        double lateralTravel = robotLateralVelocity * flightTime;
        return Math.atan2(lateralTravel, d + turretDistanceFromCenter);
    }

    /**
     * Adjust flywheel and hood offsets based on shot result.
     *
     * @param shotWasHigh True if shot was high, false if low.
     */
    public void adjustShot(boolean shotWasHigh) {
        if (shotWasHigh) {
            velocityOffset -= VELOCITY_OFFSET_STEP;
            hoodOffset -= HOOD_OFFSET_STEP;
        } else {
            velocityOffset += VELOCITY_OFFSET_STEP;
            hoodOffset += HOOD_OFFSET_STEP;
        }

        velocityOffset = Range.clip(velocityOffset, MIN_VELOCITY_OFFSET, MAX_VELOCITY_OFFSET);
        hoodOffset = Range.clip(hoodOffset, MIN_HOOD_OFFSET, MAX_HOOD_OFFSET);
    }
}
