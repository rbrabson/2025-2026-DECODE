package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.Alliance;

/**
 * ShooterController manages flywheel RPM, hood position, and predictive turret lead angle.
 */
public class ShooterController {

    private static final double ANGLE_SMOOTHING = 0.1;

    // RPM regression coefficients
    private static final double RPM_A = 0.0204772;
    private static final double RPM_B = 0.643162;
    private static final double RPM_C = 712.90909;

    // Hood regression coefficients
    private static final double HOOD_A = -2.34831e-7;
    private static final double HOOD_B = 0.0000936893;
    private static final double HOOD_C = 0.0165033;
    private static final double HOOD_D = 1.25724;

    // Flight time regression coefficients
    private static final double FLIGHT_TIME_A = 0.00001;
    private static final double FLIGHT_TIME_B = 0.001;
    private static final double FLIGHT_TIME_C = 0.1;

    private static final double RPM_MIN = 0.0;
    private static final double RPM_MAX = 1400.0;

    private static final double HOOD_MIN = 0.11;
    private static final double HOOD_MAX = 0.904;

    // Offset limits (prevents runaway drift)
    private static final double RPM_OFFSET_LIMIT = 200.0;
    private static final double HOOD_OFFSET_LIMIT = 0.1;

    private static final double MIN_FLIGHT_TIME = 0.25;
    private static final double MAX_FLIGHT_TIME = 0.7;

    /**
     * Tuning gain for velocity compensation.
     * 1.0 = physics-based
     * <1.0 = conservative
     * >1.0 = aggressive
     */
    private static final double VELOCITY_COMP_GAIN = 1.0;

    private double flywheelRPMOffset = 0.0;
    private double hoodOffset = 0.0;
    private double smoothedLeadAngle = 0.0;

    public ShooterController() {}

    /**
     * Sets the turret lead angle based on the robot's current pose and alliance target position.
     *
     * @param alliance the current alliance (RED or BLUE) to determine target position
     * @param pose     the current pose of the robot, including x, y, and heading
     * @return the ShooterController instance for chaining
     */
    @NonNull
    public ShooterController setAlliancePose(@NonNull Alliance alliance, @NonNull Pose pose) {
        double goalX = alliance.getBaseX();
        double goalY = alliance.getBaseY();

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        double angle = Math.atan2(dy, dx);
        smoothedLeadAngle = MathFunctions.normalizeAngle(angle - pose.getHeading());

        return this;
    }

    /**
     * Adjusts offsets based on shot feedback.
     */
    public void adjustShot(boolean wasHigh) {
        double deltaRPM = 20.0;
        double deltaHood = 0.01;

        flywheelRPMOffset += wasHigh ? -deltaRPM : deltaRPM;
        hoodOffset += wasHigh ? -deltaHood : deltaHood;

        flywheelRPMOffset = Range.clip(flywheelRPMOffset, -RPM_OFFSET_LIMIT, RPM_OFFSET_LIMIT);
        hoodOffset = Range.clip(hoodOffset, -HOOD_OFFSET_LIMIT, HOOD_OFFSET_LIMIT);
    }

    /**
     * Computes radial velocity (velocity toward/away from target).
     * <ul>
     * <li>Positive = moving toward target
     * <li>Negative = moving away
     * </ul>
     */
    private double getRadialVelocity(double robotVx, double robotVy, double dx, double dy) {
        double distance = Math.hypot(dx, dy);
        if (distance < 1e-6) return 0.0;

        double ux = dx / distance;
        double uy = dy / distance;

        return robotVx * ux + robotVy * uy;
    }

    /**
     * Computes effective shooting distance adjusted for robot motion.
     */
    private double getEffectiveDistance(double goalDist, double robotVx, double robotVy, double dx, double dy) {
        double radialVelocity = getRadialVelocity(robotVx, robotVy, dx, dy);
        double flightTime = getFlightTime(goalDist);
        double effectiveDist = goalDist + VELOCITY_COMP_GAIN * radialVelocity * flightTime;

        return Math.max(0.0, effectiveDist);
    }

    /**
     * Predicts flywheel RPM with velocity compensation.
     */
    public double getFlywheelRPM(double goalDist, double robotVx, double robotVy, double dx, double dy) {
        double effectiveDist = getEffectiveDistance(goalDist, robotVx, robotVy, dx, dy);

        double rpm = RPM_A * effectiveDist * effectiveDist
                + RPM_B * effectiveDist
                + RPM_C;

        rpm = Range.clip(rpm, RPM_MIN, RPM_MAX);
        rpm += flywheelRPMOffset;

        return rpm;
    }

    /**
     * Predicts hood position with velocity compensation.
     */
    public double getHoodPosition(double goalDist, double robotVx, double robotVy, double dx, double dy) {
        double effectiveDist = getEffectiveDistance(goalDist, robotVx, robotVy, dx, dy);

        double position = HOOD_A * Math.pow(effectiveDist, 3)
                + HOOD_B * Math.pow(effectiveDist, 2)
                - HOOD_C * effectiveDist
                + HOOD_D;

        position = Range.clip(position, HOOD_MIN, HOOD_MAX);
        position += hoodOffset;

        return position;
    }

    /**
     * Predicts turret lead angle based on motion and target prediction.
     */
    public double getTurretLeadAngle(double x, double y, double heading, double fieldVx, double fieldVy, double angularVel, double targetX, double targetY) {
        double dx = targetX - x;
        double dy = targetY - y;

        double distance = Math.hypot(dx, dy);
        double flightTime = getFlightTime(distance);

        double futureX = targetX + fieldVx * flightTime;
        double futureY = targetY + fieldVy * flightTime;

        double pdx = futureX - x;
        double pdy = futureY - y;

        double angleToTarget = Math.atan2(pdy, pdx);

        double leadAngle = MathFunctions.normalizeAngle(angleToTarget - heading);

        // Rotational compensation
        leadAngle += angularVel * flightTime;

        // Smooth output
        smoothedLeadAngle += ANGLE_SMOOTHING * (leadAngle - smoothedLeadAngle);

        return smoothedLeadAngle;
    }

    /**
     * Predicts projectile flight time.
     */
    private double getFlightTime(double goalDist) {
        double flightTime = FLIGHT_TIME_A * goalDist * goalDist
                + FLIGHT_TIME_B * goalDist
                + FLIGHT_TIME_C;

        return Range.clip(flightTime, MIN_FLIGHT_TIME, MAX_FLIGHT_TIME);
    }
}
