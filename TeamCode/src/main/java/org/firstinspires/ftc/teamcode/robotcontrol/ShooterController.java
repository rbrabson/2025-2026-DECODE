package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.interplut.InterpLUT;

import org.firstinspires.ftc.teamcode.decode.Alliance;

/**
 * ShooterController manages flywheel RPM, hood position, and predictive turret lead angle
 * based on distance to target and feedback from previous shots.
 */
public class ShooterController {
    private static final double ANGLE_SMOOTHING = 0.1;

    // Tunable RPM quadratic regression coefficients
    private static final double RPM_COEFFICIENT_A = 0.0204772;
    private static final double RPM_COEFFICIENT_B = 0.643162;
    private static final double RPM_COEFFICIENT_C = 712.90909;

    // Tunable hood position cubic regression coefficients
    private static final double HOOD_COEFFICIENT_A = -2.34831e-7;
    private static final double HOOD_COEFFICIENT_B = 0.0000936893;
    private static final double HOOD_COEFFICIENT_C = 0.0165033;
    private static final double HOOD_COEFFICIENT_D = 1.25724;

    private static final double RPM_MIN = 0.0;
    private static final double RPM_MAX = 1400.0;

    private static final double HOOD_MIN = 0.11;
    private static final double HOOD_MAX = 0.904;

    private double flywheelRPMOffset = 0.0;
    private double hoodOffset = 0.0;
    private double smoothedLeadAngle = 0.0;

    /**
     * Initializes the ShooterController with predefined LUTs for flywheel RPM, hood position,
     * and flight time.
     */
    public ShooterController() {
    }

    /**
     * Sets the alliance and current pose to calculate the distance and angle to the goal, which are
     * used to initialize the smoothed RPM and lead angle. This allows the controller to start
     * with reasonable values based on the robot's initial position relative to the target, improving
     * the accuracy of the first few shots before feedback adjustments take effect.
     *
     * @param alliance the alliance the robot is on, used to determine the goal position for calculations
     * @param pose     the current pose of the robot, used to calculate the distance and angle to
     *                 the goal for setting
     * @return the ShooterController instance, allowing for method chaining if desired
     */
    @NonNull
    public ShooterController setAlliancePose(@NonNull Alliance alliance, @NonNull Pose pose) {
        double goalX = alliance.getBaseX();
        double goalY = alliance.getBaseY();

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        double distance = Math.hypot(dx, dy);

        this.smoothedLeadAngle = Math.atan2(dy, dx) - pose.getHeading();;
        return this;
    }

    /**
     * Adjusts flywheel RPM and hood position offsets based on whether the last shot was high or low.
     *
     * @param wasHigh true if the last shot was high, false if it was low. This will adjust the
     *                offsets to try to correct for the error.
     */
    public void adjustShot(boolean wasHigh) {
        double deltaRPM = 20.0;
        double deltaHood = 0.01;
        flywheelRPMOffset += wasHigh ? -deltaRPM : deltaRPM;
        hoodOffset += wasHigh ? -deltaHood : deltaHood;
    }

    /**
     * Returns the target flywheel RPM for a given distance, applying smoothing to prevent abrupt changes.
     *
     * @param goalDist the distance to the target, used in the quadratic regression to calculate the
     *                 flywheel RPM.
     * @return the smoothed target flywheel RPM to use for shooting at the given distance
     */
    public double getFlywheelRPM(double goalDist) {
        double rpm = RPM_COEFFICIENT_A * Math.pow(goalDist, 2)
                + RPM_COEFFICIENT_B * goalDist
                + RPM_COEFFICIENT_C;
        rpm = Range.clip(rpm, RPM_MIN, RPM_MAX);
        rpm += flywheelRPMOffset;
        return rpm;
    }

    /**
     * Returns the target hood position for a given distance, applying the current hood offset.
     *
     * @param goalDist the distance to the target, used in the cubic regression to calculate the
     *                 hood position.
     * @return the target hood position to use for shooting at the given distance
     */
    public double getHoodPosition(double goalDist) {
        double position = HOOD_COEFFICIENT_A * Math.pow(goalDist, 3)
                + HOOD_COEFFICIENT_B * Math.pow(goalDist, 2)
                - HOOD_COEFFICIENT_C * goalDist
                + HOOD_COEFFICIENT_D;
        position = Range.clip(position, HOOD_MIN, HOOD_MAX);
        position += hoodOffset;
        return position;
    }

    /**
     * Returns predictive turret lead angle based on velocity and angular motion
     *
     * @param x          current x position of the robot
     * @param y          current y position of the robot
     * @param heading    current heading of the robot in radians
     * @param fieldVx    current velocity of the target in the x direction (field frame)
     * @param fieldVy    current velocity of the target in the y direction (field frame)
     * @param angularVel current angular velocity of the target in radians per second
     * @param targetX    current x position of the target
     * @param targetY    current y position of the target
     * @return the predictive lead angle to aim the turret, in radians, which accounts for the
     *         target's motion and the projectile's flight time, allowing for more accurate shots
     *         against moving targets.
     */
    public double getTurretLeadAngle(double x, double y, double heading, double fieldVx, double fieldVy, double angularVel, double targetX, double targetY) {
        double dx = targetX - x;
        double dy = targetY - y;
        double distance = Math.hypot(dx, dy);

        // Predict future target position in field frame
        double futureX = targetX + fieldVx;
        double futureY = targetY + fieldVy;

        double pdx = futureX - x;
        double pdy = futureY - y;

        double angleToTarget = Math.atan2(pdy, pdx);
        double leadAngle = angleToTarget - heading;

        // Add rotational compensation
        leadAngle += angularVel * 0.5;

        // Smooth the lead angle
        smoothedLeadAngle += ANGLE_SMOOTHING * (leadAngle - smoothedLeadAngle);
        return smoothedLeadAngle;
    }
}
