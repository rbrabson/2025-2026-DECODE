package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.Alliance;

/**
 * ShooterController manages flywheel RPM, hood position, and predictive turret lead angle
 * based on distance to target and feedback from previous shots.
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

    // Flight time model (tune this)
    private static final double FLIGHT_TIME_PER_UNIT = 0.0025;

    private double flywheelRPMOffset = 0.0;
    private double hoodOffset = 0.0;
    private double smoothedLeadAngle = 0.0;

    public ShooterController() {
    }

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

        // Clamp to prevent runaway tuning
        flywheelRPMOffset = Range.clip(flywheelRPMOffset, -RPM_OFFSET_LIMIT, RPM_OFFSET_LIMIT);
        hoodOffset = Range.clip(hoodOffset, -HOOD_OFFSET_LIMIT, HOOD_OFFSET_LIMIT);
    }

    /**
     * Predicts flywheel RPM based on distance using a quadratic regression model.
     *
     * @param goalDist Distance to target in inches
     * @return Predicted flywheel RPM, adjusted by offset and clamped to limits
     */
    public double getFlywheelRPM(double goalDist) {
        double rpm = RPM_A * goalDist * goalDist
                + RPM_B * goalDist
                + RPM_C;

        rpm = Range.clip(rpm, RPM_MIN, RPM_MAX);
        rpm += flywheelRPMOffset;

        return rpm;
    }

    /**
     * Predicts hood position based on distance using a cubic regression model.
     *
     * @param goalDist Distance to target in inches
     * @return Predicted hood position (0.0 to 1.0), adjusted by offset and clamped to limits
     */
    public double getHoodPosition(double goalDist) {
        double position = HOOD_A * Math.pow(goalDist, 3)
                + HOOD_B * Math.pow(goalDist, 2)
                - HOOD_C * goalDist
                + HOOD_D;

        position = Range.clip(position, HOOD_MIN, HOOD_MAX);
        position += hoodOffset;

        return position;
    }


    /**
     * Predicts turret lead angle based on current robot state and target motion.
     *
     * @param x          Current robot X position
     * @param y          Current robot Y position
     * @param heading    Current robot heading in radians
     * @param fieldVx    Target velocity in X direction (field-relative)
     * @param fieldVy    Target velocity in Y direction (field-relative)
     * @param angularVel Target angular velocity (radians/sec)
     * @param targetX    Current target X position
     * @param targetY    Current target Y position
     * @return Predicted turret lead angle in radians, smoothed over time
     */
    public double getTurretLeadAngle(
            double x,
            double y,
            double heading,
            double fieldVx,
            double fieldVy,
            double angularVel,
            double targetX,
            double targetY
    ) {

        // Current vector to target
        double dx = targetX - x;
        double dy = targetY - y;
        double distance = Math.hypot(dx, dy);

        // Estimate projectile flight time
        double flightTime = getFlightTime(distance);

        // Predict future target position
        double futureX = targetX + fieldVx * flightTime;
        double futureY = targetY + fieldVy * flightTime;

        // Recompute vector to predicted position
        double pdx = futureX - x;
        double pdy = futureY - y;

        double angleToTarget = Math.atan2(pdy, pdx);

        // Normalize relative angle
        double leadAngle = MathFunctions.normalizeAngle(angleToTarget - heading);

        // Add rotational compensation (scaled properly by time)
        leadAngle += angularVel * flightTime;

        // Smooth output
        smoothedLeadAngle += ANGLE_SMOOTHING * (leadAngle - smoothedLeadAngle);

        return smoothedLeadAngle;
    }

    /**
     * Predicts projectile flight time based on distance using a quadratic regression model.
     *
     * @param goalDist Distance to target in inches
     * @return Predicted flight time in seconds
     */
    private double getFlightTime(double goalDist) {
        double flightTime = FLIGHT_TIME_A * Math.pow(goalDist, 2)
                + FLIGHT_TIME_B * goalDist
                + FLIGHT_TIME_C;
        flightTime = Range.clip(flightTime, MIN_FLIGHT_TIME, MAX_FLIGHT_TIME);
        return flightTime;
    }
}
