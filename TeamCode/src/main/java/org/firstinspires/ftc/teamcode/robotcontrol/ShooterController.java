package org.firstinspires.ftc.teamcode.robotcontrol;

import com.rbrabson.control.interplut.InterpLUT;

/**
 * ShooterController manages flywheel RPM, hood position, and predictive turret lead angle
 * based on distance to target and feedback from previous shots.
 */
public class ShooterController {
    private static final double RPM_SMOOTHING = 0.15;
    private static final double ANGLE_SMOOTHING = 0.1;
    private final InterpLUT flywheelLUT;
    private final InterpLUT hoodLUT;
    private final InterpLUT flightTimeLUT;
    private double flywheelRPMOffset = 0.0;
    private double hoodOffset = 0.0;
    private double smoothedRPM = 0.0;
    private double smoothedLeadAngle = 0.0;

    /**
     * Initializes the ShooterController with predefined LUTs for flywheel RPM, hood position,
     * and flight time.
     */
    public ShooterController() {
        this.flywheelLUT = getFlywheelLUT();
        this.hoodLUT = getHoodLUT();
        this.flightTimeLUT = getFlightTimeLUT();
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
     * @param distance the distance to the target, used to look up the base RPM from the LUT
     * @return the smoothed target flywheel RPM to use for shooting at the given distance
     */
    public double getFlywheelRPM(double distance) {
        double targetRPM = flywheelLUT.get(distance) + flywheelRPMOffset;
        smoothedRPM += RPM_SMOOTHING * (targetRPM - smoothedRPM);
        return smoothedRPM;
    }

    /**
     * Returns the target hood position for a given distance, applying the current hood offset.
     *
     * @param distance the distance to the target, used to look up the base hood position from the LUT
     * @return the target hood position to use for shooting at the given distance
     */
    public double getHoodPosition(double distance) {
        return hoodLUT.get(distance) + hoodOffset;
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

        // Flight time from LUT
        double flightTime = flightTimeLUT.get(distance);

        // Predict future target position in field frame
        double futureX = targetX + fieldVx * flightTime;
        double futureY = targetY + fieldVy * flightTime;

        double pdx = futureX - x;
        double pdy = futureY - y;

        double angleToTarget = Math.atan2(pdy, pdx);
        double leadAngle = angleToTarget - heading;

        // Add rotational compensation
        leadAngle += angularVel * flightTime * 0.5;

        // Smooth the lead angle
        smoothedLeadAngle += ANGLE_SMOOTHING * (leadAngle - smoothedLeadAngle);
        return smoothedLeadAngle;
    }

    /**
     * Defines the flywheel RPM LUT based on distance to target.
     *
     * @return an InterpLUT mapping distance to flywheel RPM, which can be used to determine the
     *         base RPM for shooting at different distances.
     */
    private InterpLUT getFlywheelLUT() {
        return new InterpLUT()
                .withPoint(20, 500)
                .withPoint(50, 1170)
                .withPoint(75, 1750)
                .withPoint(100, 2000)
                .build();
    }

    /**
     * Defines the hood position LUT based on distance to target.
     *
     * @return an InterpLUT mapping distance to hood position, which can be used to determine the base
     *         hood angle for shooting at different distances.
     */
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
     * Defines the flight time LUT based on distance to target, which can be used for predictive aiming.
     *
     * @return an InterpLUT mapping distance to estimated flight time, which can be used to predict
     *         where the target will be when the projectile arrives, allowing for lead compensation
     *         in the turret aiming.
     */
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
