package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.Alliance;

/**
 * ShooterController manages flywheel RPM, hood position, and predictive turret lead angle.
 * <p>
 * Features:
 * - Distance-based regression for RPM + hood
 * - Radial velocity compensation (forward/back motion)
 * - Lateral velocity compensation (strafing)
 * - Predictive turret aiming (robot + target motion)
 * - Shot feedback auto-tuning
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

    private static final double RPM_OFFSET_LIMIT = 200.0;
    private static final double HOOD_OFFSET_LIMIT = 0.1;

    private static final double MIN_FLIGHT_TIME = 0.25;
    private static final double MAX_FLIGHT_TIME = 0.7;

    // Gains for radial (forward/back) and lateral (strafing) compensation
    private static final double VELOCITY_COMP_GAIN = 1.0;
    private static final double STRAFE_COMP_GAIN = 1.0;

    private static final double EPSILON = 1e-6;

    private double flywheelRPMOffset = 0.0;
    private double hoodOffset = 0.0;
    private double smoothedLeadAngle = 0.0;

    public ShooterController() {
    }

    /**
     * Sets the alliance-specific pose for turret lead angle calculation.
     *
     * @param alliance The alliance (RED or BLUE) to determine field orientation
     * @param pose     The current robot pose (x, y, heading) for calculating lead angle
     * @return this for chaining
     */
    @NonNull
    public ShooterController setAlliancePose(@NonNull Alliance alliance, @NonNull Pose pose) {
        double dx = alliance.getBaseX() - pose.getX();
        double dy = alliance.getBaseY() - pose.getY();

        double angle = Math.atan2(dy, dx);
        smoothedLeadAngle = MathFunctions.normalizeAngle(angle - pose.getHeading());

        return this;
    }

    /**
     * Adjusts flywheel RPM and hood position based on shot feedback.
     *
     * @param wasHigh true if the shot was too high, false if too low.
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
     * Predicts flywheel RPM with velocity compensation. The effective distance is calculated by
     * adjusting the actual distance to the target based on the robot's radial velocity towards or
     * away from the target, multiplied by the predicted flight time of the projectile.
     *
     * @param dx      difference in x position to the target
     * @param dy      difference in y position to the target
     * @param robotVx robot's velocity in the x direction (forward)
     * @param robotVy robot's velocity in the y direction (lateral)
     * @return the predicted flywheel RPM needed to hit the target, adjusted for the robot's motion.
     */
    public double getFlywheelRPM(double dx, double dy, double robotVx, double robotVy) {
        double d = getEffectiveDistance(dx, dy, robotVx, robotVy);
        double rpm = RPM_A * d * d + RPM_B * d + RPM_C;

        return Range.clip(rpm + flywheelRPMOffset, RPM_MIN, RPM_MAX);
    }

    /**
     * Calculates an effective distance to the target that accounts for the robot's motion.
     * This is used to adjust the flywheel RPM and hood position to compensate for the fact that
     * the robot is moving while shooting.
     *
     * @param dx      difference in x position to the target
     * @param dy      difference in y position to the target
     * @param robotVx robot's velocity in the x direction (forward)
     * @param robotVy robot's velocity in the y direction (lateral)
     * @return the effective distance to the target that accounts for the robot's motion,
     *         which can be greater or less than the actual distance depending on the direction
     *         of motion
     */
    private double getEffectiveDistance(double dx, double dy, double robotVx, double robotVy) {
        double dist = Math.hypot(dx, dy);
        double radialVel = getRadialVelocity(robotVx, robotVy, dx, dy);
        double flightTime = getFlightTime(dist);
        double effectiveDist = dist + VELOCITY_COMP_GAIN * radialVel * flightTime;

        return Math.max(0.0, effectiveDist);
    }

    /**
     * Calculates the radial velocity component of the robot's motion towards/away from the target.
     *
     * @param robotVx robot's velocity in the x direction (forward)
     * @param robotVy robot's velocity in the y direction (lateral)
     * @param dx      difference in x position to the target
     * @param dy      difference in y position to the target
     * @return the radial velocity component (positive if moving towards the target, negative if moving away)
     */
    private double getRadialVelocity(double robotVx, double robotVy, double dx, double dy) {
        double dist = Math.hypot(dx, dy);
        if (dist < EPSILON) return 0.0;
        return (robotVx * dx + robotVy * dy) / dist;
    }

    /**
     * Predicts the flight time of the projectile based on the distance to the target. The flight
     * time is estimated using a quadratic regression based on empirical data of how long it takes
     * for the projectile to reach targets at various distances.
     *
     * @param dist the distance to the target, which is used to predict the flight time of the projectile.
     * @return the predicted flight time in seconds, which is used for compensating the flywheel RPM,
     *         hood position, and turret lead angle based on the robot's motion and the target's
     *         motion. The flight time is clipped to be within defined minimum and maximum limits
     *         to ensure reasonable predictions.
     */
    private double getFlightTime(double dist) {
        double t = FLIGHT_TIME_A * dist * dist + FLIGHT_TIME_B * dist + FLIGHT_TIME_C;
        return Range.clip(t, MIN_FLIGHT_TIME, MAX_FLIGHT_TIME);
    }

    /**
     * Predicts hood position with velocity compensation. Similar to the flywheel RPM prediction,
     * the effective distance is calculated by adjusting the actual distance to the target based
     * on the robot's radial velocity towards or away from the target, multiplied by the predicted
     * flight time of the projectile. This allows the shooter to compensate for the fact that the
     * robot is moving while shooting, which can affect the required hood position to hit the
     * target accurately.
     *
     * @param dx      difference in x position to the target
     * @param dy      difference in y position to the target
     * @param robotVx robot's velocity in the x direction (forward)
     * @param robotVy robot's velocity in the y direction (lateral)
     * @return the predicted hood position needed to hit the target, adjusted for the robot's motion.
     */
    public double getHoodPosition(double dx, double dy, double robotVx, double robotVy) {
        double d = getEffectiveDistance(dx, dy, robotVx, robotVy);
        double pos = HOOD_A * d * d * d + HOOD_B * d * d - HOOD_C * d + HOOD_D;
        return Range.clip(pos + hoodOffset, HOOD_MIN, HOOD_MAX);
    }

    /**
     * Predicts turret lead angle with strafing compensation. This method calculates the angle at
     * which the turret should aim to hit a moving target, taking into account both the robot's and
     * the target's velocities, as well as the time it will take for the projectile to reach the
     * target. The method predicts the future positions of both the robot and the target after the
     * flight time of the projectile, and then calculates the angle from the future robot position
     * to the future target position. This allows for accurate aiming even when both the robot and
     * the target are in motion, including strafing (lateral movement).
     *
     * @param x          current x position of the robot
     * @param y          current y position of the robot
     * @param heading    current heading of the robot in radians
     * @param robotVx    robot's velocity in the x direction (forward)
     * @param robotVy    robot's velocity in the y direction (lateral)
     * @param targetVx   target's velocity in the x direction
     * @param targetVy   target's velocity in the y direction
     * @param angularVel robot's angular velocity (rotation rate) in radians per second, used for rotational compensation
     * @param targetX    current x position of the target
     * @param targetY    current y position of the target
     * @return the predicted turret lead angle in radians, which is the angle the turret should
     *         aim to hit the moving target, adjusted for the robot's motion, target's motion,
     *         and rotational compensation.
     */
    public double getTurretLeadAngle(double x, double y, double heading, double robotVx, double robotVy, double targetVx, double targetVy, double angularVel, double targetX, double targetY) {
        double dx = targetX - x;
        double dy = targetY - y;

        double dist = Math.hypot(dx, dy);
        double t = getFlightTime(dist);

        // Predict future robot position (THIS is the strafing fix)
        double futureRobotX = x + STRAFE_COMP_GAIN * robotVx * t;
        double futureRobotY = y + STRAFE_COMP_GAIN * robotVy * t;

        // Predict future target position
        double futureTargetX = targetX + targetVx * t;
        double futureTargetY = targetY + targetVy * t;

        // Aim from future robot → future target
        double aimDx = futureTargetX - futureRobotX;
        double aimDy = futureTargetY - futureRobotY;

        double angle = Math.atan2(aimDy, aimDx);

        double lead = MathFunctions.normalizeAngle(angle - heading);

        // rotational compensation
        lead += angularVel * t;

        // smoothing
        smoothedLeadAngle += ANGLE_SMOOTHING * (lead - smoothedLeadAngle);

        return smoothedLeadAngle;
    }
}
