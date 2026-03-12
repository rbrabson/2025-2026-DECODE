package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing;

import com.rbrabson.behave.Status;

/**
 * This interface represents an autonomous routine for the robot. It has a single method,
 * update(), which is called repeatedly in a loop during the autonomous period. The update()
 * method should return a Status value indicating whether the autonomous routine is still running,
 * has succeeded, or has failed. This allows the main OpMode to manage the flow of the autonomous
 * routine and determine when it has completed or if it needs to be stopped due to an error.
 */
public interface AutonomousPathing {
    Status update();
}
