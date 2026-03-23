package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpMode;

/**
 * This OpMode is used for tuning the shooter mechanism. It allows the user to test and adjust the
 * shooter's performance using the RED alliance goal as a reference. The robot is set to manual
 * control mode for the shooter, enabling precise adjustments of the hood position and flywheel RPm.
 */
@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuning extends TeleOpMode {
    /**
     * Start the robot in manual shooting mode
     */
    @Override
    public void start() {
        Robot.reset();
        Robot robot = Robot.getInstance(hardwareMap, telemetry);
        super.start();
        robot.shooter.setManualControl(true);
        robot.shooter.setHoodPosition(0.0);
        robot.shooter.setFlywheelRPM(0.0);
    }

    /**
     * Tune using the RED alliance goal
     * @return The alliance that the robot is on (RED).
     */
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
