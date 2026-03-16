package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Alliance;

/**
 * Teleop for the red alliance.
 */
@TeleOp(name = "Red Teleop", group = "Active")
public class RedTeleop extends PedroPathingTeleOp {

    /**
     * Returns the alliance that the robot is on, which is RED for this teleop mode.
     *
     * @return The alliance that the robot is on (RED).
     */
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
