package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * An interface for mechanisms. A mechanism is a subsystem of the robot that performs a specific
 * function, such as driving, shooting, or intaking. The update method is called every loop to
 * update the mechanism's state.
 */
public interface Mechanism {
    /**
     * Updates the mechanism's state. This method is called every loop to update the mechanism's state.
     */
    void update();
}
