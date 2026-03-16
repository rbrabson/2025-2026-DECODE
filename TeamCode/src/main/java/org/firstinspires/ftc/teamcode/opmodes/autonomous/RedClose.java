package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.AutonomousPathing;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.RedClosePathing;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;

/**
 * This is the OpMode for the Red Close autonomous routine. It initializes the robot, follower, and path,
 * and runs the autonomous actions in the loop. The robot will start at a specific pose, follow a series
 * of paths to score preloads and pick up additional game elements, and will also check for motif detection
 * to adjust its behavior accordingly.
 */
@Autonomous(name = "Red Close", group = "Active")
public class RedClose extends AutonomousOpMode {
    /**
     * This method returns the autonomous path to be followed during this routine.
     *
     * @return an instance of the RedCloseAutonomous class.
     */
    @Override
    protected AutonomousPathing getPath() {
        return new RedClosePathing(follower, robot.intake, robot.shooter, robot.transfer);
    }

    /**
     * This method returns the flywheel RPM for the opmode.
     *
     * @return the flywheel RPM to be set for this autonomous routine. In this case, it
     *         returns the RPM for close-range shooting.
     */
    @Override
    protected double getFlywheelRPM() {
        return Flywheel.RPM_AUTON_CLOSE;
    }

    /**
     * This method returns the starting pose for the robot.
     *
     * @return the starting pose of the robot for this autonomous routine.
     */
    @Override
    protected Pose getStartingPose() {
        return RedClosePathing.startPose;
    }

    /**
     * This method returns the team's alliance.
     *
     * @return the base for this autonomous routine, which is used to set the turret base values for
     * the shooter.
     */
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    /**
     * This method switches the Limelight camera to the pipeline used to detect the red goal.
     */
    @Override
    protected void switchToShootingPipeline() {
        robot.limelight.switchToRedGoal();
    }
}
