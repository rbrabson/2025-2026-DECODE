package org.firstinspires.ftc.teamcode.robotcontrol;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Flywheel;

/**
 * ShooterController uses cubic regression coefficients to predict flywheel RPM and
 * hood position * based on distance. It also includes a low-pass filter for distance measurements
 * and an * automatic shot tuning mechanism.
 */
public class ShooterController {
    // Distance filter
    private static final double FILTER_ALPHA = 0.2;

    // Distance domain used for regression validity
    private static final double MIN_SHOT_DISTANCE = 0.0;
    private static final double MAX_SHOT_DISTANCE = 200.0;

    // Automatic shot tuning in RPM
    private static final double RPM_OFFSET_STEP = 50.0;
    private static final double MIN_RPM_OFFSET = -Flywheel.RPM_MAX / 2.0;
    private static final double MAX_RPM_OFFSET = Flywheel.RPM_MAX / 2.0;

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
    private static final double MIN_HOOD = 0.0;
    private static final double MAX_HOOD = 0.8;

    // Automatic shot tuning
    private static final double HOOD_OFFSET_STEP = 0.01;
    private static final double MIN_HOOD_OFFSET = -0.20;
    private static final double MAX_HOOD_OFFSET = 0.20;

    // Artifact flight constant (seconds per inch)
    private static final double FLIGHT_TIME_PER_INCH = 0.002;

    // Flywheel RPM correction factor for robot motion compensation
    public static final double FLYWHEEL_CORRECTION = 5.0;

    private double rpmOffset = 0.0;
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
     * Calculate flywheel RPM for the given distance and robot velocity toward the target, applying
     * motion compensation and tuning offsets.
     *
     * @param distance                  The distance to the target (in inches).
     * @param robotVelocityTowardTarget Robot velocity toward the target (in/s). Positive means
     *                                  moving closer, negative means moving away.
     * @return Calculated flywheel RPM, clamped to limits.
     */
    public double getFlywheelRPM(double distance, double robotVelocityTowardTarget) {
        double d = Range.clip(distance, MIN_SHOT_DISTANCE, MAX_SHOT_DISTANCE);
        double rpm = ((FA * d + FB) * d + FC) * d + FD; // Assume coefficients now output RPM

        rpm -= robotVelocityTowardTarget * FLYWHEEL_CORRECTION;
        rpm += rpmOffset;
        return Range.clip(rpm, -Flywheel.RPM_MAX, Flywheel.RPM_MAX);
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
     * Adjust the RPM and hood position offsets based on whether the shot was high or low.
     * This method is called after each shot to fine-tune the shooting parameters for future shots.
     *
     * @param shotWasHigh True if the shot was high (overshot), false if it was low (undershot).
     */
    public void adjustShot(boolean shotWasHigh) {
        if (shotWasHigh) {
            rpmOffset -= RPM_OFFSET_STEP;
            hoodOffset -= HOOD_OFFSET_STEP;
        } else {
            rpmOffset += RPM_OFFSET_STEP;
            hoodOffset += HOOD_OFFSET_STEP;
        }

        rpmOffset = Range.clip(rpmOffset, MIN_RPM_OFFSET, MAX_RPM_OFFSET);
        hoodOffset = Range.clip(hoodOffset, MIN_HOOD_OFFSET, MAX_HOOD_OFFSET);
    }
}
