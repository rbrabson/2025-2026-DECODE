package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

/**
 * Interface for drive mechanisms. This interface extends the base Mechanism interface and can be
 * implemented by any drive system (e.g., mecanum, tank, swerve) to ensure consistent control and
 * integration with the robot's overall mechanism management.
 */
public interface Drive extends Mechanism {
    /**
     * Gets the current localizer associated with this drive mechanism.
     *
     * @return The current Localizer instance used for tracking the robot's position and orientation.
     */
    Localizer getLocalizer();

    /**
     * Gets the current pose of the robot as determined by the localizer.
     *
     * @return The current Pose of the robot, including its position (x, y) and heading (theta).
     */
    Pose getPose();

    /**
     * Drives the robot using the specified forward, strafe, and turn values.
     *
     * @param forward The forward/backward speed of the robot. Positive values move the robot forward, while negative values move it backward.
     * @param strafe  The left/right speed of the robot. Positive values move the robot to the right, while negative values move it to the left.
     * @param turn    The rotational speed of the robot. Positive values turn the robot to the right, while negative values turn it to the left.
     */
    void drive(double forward, double strafe, double turn);

    /**
     * Drives the robot to a specific target pose using the localizer for feedback.
     * This method should implement a control algorithm (e.g., PID) to move the robot towards the
     * target pose while minimizing error.
     *
     * @param target The target Pose that the robot should drive to, including the desired
     *               position (x, y) and heading (theta).
     */
    void driveToPose(Pose target);
}
