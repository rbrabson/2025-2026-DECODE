package org.firstinspires.ftc.teamcode.robotcontrol;

import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.interplut.InterpLUT;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;

/**
 * ShooterController class calculates flywheel RPM, hood position, and turret lead angle
 * based on distance to the target and robot velocity. Includes a feedback mechanism and
 * improved turret lead using a flight-time LUT.
 * <p>
 * Key features:
 * <ul>
 *   <li>Distance filtering with low-pass filter to smooth measurements.
 *   <li>Flywheel RPM adjusted for robot forward velocity.
 *   <li>Hood position adjustable via LUT and offset for tuning.
 *   <li>Turret lead angle uses flight-time LUT and clamps to a maximum safe angle.
 *   <li>Feedback system allows incremental adjustment of RPM and hood per shot.
 * </ul>
 */
public class ShooterController {
    private static final double FILTER_ALPHA = 0.2;
    private static final double MIN_SHOT_DISTANCE = 0.0;
    private static final double MAX_SHOT_DISTANCE = 200.0;

    private static final double RPM_OFFSET_STEP = 50.0;
    private static final double MIN_RPM_OFFSET = -Flywheel.RPM_MAX / 2.0;
    private static final double MAX_RPM_OFFSET = Flywheel.RPM_MAX / 2.0;

    private static final double MIN_HOOD = 0.0;
    private static final double MAX_HOOD = 0.8;
    private static final double HOOD_OFFSET_STEP = 0.015;
    private static final double MIN_HOOD_OFFSET = -0.20;
    private static final double MAX_HOOD_OFFSET = 0.20;

    public static final double FLYWHEEL_CORRECTION = 5.0;
    private static final double MAX_VELOCITY = 60.0; // inches/sec clamp for velocity inputs
    private static final double MAX_LEAD_RAD = Math.toRadians(10.0); // ±10° max turret lead

    private double rpmOffset = 0.0;
    private double hoodOffset = 0.0;
    private double filteredDistance = 0.0;
    private boolean distanceFilterInitialized = false;

    // Interpolation LUT tables
    private final InterpLUT flywheelLUT;
    private final InterpLUT hoodLUT;
    private final InterpLUT flightTimeLUT;

    public ShooterController() {
        flywheelLUT = getFlywheelLUT();
        hoodLUT = getHoodLUT();
        flightTimeLUT = getFlightTimeLUT();
    }

    /**
     * Updates and filters raw distance measurement using a low-pass filter.
     *
     * @param measuredDistance Raw distance to target (inches)
     * @return Filtered distance to target
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
     * Returns the calculated flywheel RPM for shooting based on distance and robot forward velocity.
     * RPM is adjusted for robot movement and user feedback offsets.
     *
     * @param distance Filtered distance to target (inches)
     * @param robotVelocityTowardTarget Robot velocity toward the target (inches/sec)
     * @return Flywheel RPM, clipped to safe range
     */
    public double getFlywheelRPM(double distance, double robotVelocityTowardTarget) {
        double d = Range.clip(distance, 20.0, 100.0); // LUT range
        double rpm = flywheelLUT.get(d);

        robotVelocityTowardTarget = Range.clip(robotVelocityTowardTarget, -MAX_VELOCITY, MAX_VELOCITY);

        rpm -= robotVelocityTowardTarget * FLYWHEEL_CORRECTION;
        rpm += rpmOffset;

        return Range.clip(rpm, -Flywheel.RPM_MAX, Flywheel.RPM_MAX);
    }

    /**
     * Returns the hood position based on distance and user offset.
     *
     * @param distance Filtered distance to target (inches)
     * @return Servo position (0.0–1.0), clipped to min/max limits
     */
    public double getHoodPosition(double distance) {
        double d = Range.clip(distance, 0.0, 100.0); // LUT range
        double hood = hoodLUT.get(d);
        hood += hoodOffset;
        return Range.clip(hood, MIN_HOOD, MAX_HOOD);
    }

    /**
     * Returns the turret lead angle needed to compensate for robot lateral movement.
     * Uses a flight-time LUT for improved accuracy, and clamps to a maximum safe lead angle.
     *
     * @param distance Distance to target (inches)
     * @param robotLateralVelocity Robot lateral velocity (inches/sec)
     * @return Lead angle in radians, clipped to ±MAX_LEAD_RAD
     */
    public double getTurretLeadAngle(double distance, double robotLateralVelocity) {
        double d = Math.max(1e-6, distance);

        robotLateralVelocity = Range.clip(robotLateralVelocity, -MAX_VELOCITY, MAX_VELOCITY);

        double flightTime = flightTimeLUT.get(d);
        double lateralTravel = robotLateralVelocity * flightTime;

        double leadAngle = Math.atan2(lateralTravel, d);

        // Clamp lead angle to protect turret
        return Range.clip(leadAngle, -MAX_LEAD_RAD, MAX_LEAD_RAD);
    }

    /**
     * Updates flywheel RPM and hood offsets based on last shot.
     * Positive offset moves shot higher; negative moves lower.
     *
     * @param shotWasHigh True if last shot went too high, false if too low
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

    /**
     * Flywheel LUT (distance → RPM)
     *
     * @return InterpLUT mapping distance to flywheel RPM, built from empirical data and tuning
     * */
    private InterpLUT getFlywheelLUT() {
        return new InterpLUT()
                .withPoint(20, 500)
                .withPoint(50, 1170)
                .withPoint(75, 1750)
                .withPoint(100, 2000)
                .build();
    }

    /**
     * Hood LUT (distance → servo position)
     *
     * @return InterpLUT mapping distance to hood servo position, built from empirical data and tuning
     * */
    private InterpLUT getHoodLUT() {
        return new InterpLUT()
                .withPoint(0, 0.0)
                .withPoint(20, 0.0)
                .withPoint(50, 0.1)
                .withPoint(75, 0.2)
                .withPoint(100, 0.4)
                .build();
    }

    /**
     * Flight-time LUT (distance → seconds for projectile to reach target)
     *
     * @return InterpLUT mapping distance to flight time, built from projectile motion equations and tuning
     * */
    private InterpLUT getFlightTimeLUT() {
        return new InterpLUT()
                .withPoint(20, 0.05)
                .withPoint(40, 0.08)
                .withPoint(60, 0.11)
                .withPoint(80, 0.15)
                .withPoint(100, 0.20)
                .build();
    }
}
