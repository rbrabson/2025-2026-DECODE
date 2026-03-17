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
}
