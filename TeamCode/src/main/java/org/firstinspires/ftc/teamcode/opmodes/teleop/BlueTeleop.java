package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.Alliance;

/**
 * Teleop for the blue alliance.
 */
@TeleOp(name = "Blue Teleop", group = "Active")
public class BlueTeleop extends TeleOpMode {

    /**
     * Returns the alliance that the robot is on, which is BLUE for this teleop mode.
     *
     * @return The alliance that the robot is on (BLUE).
     */
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
