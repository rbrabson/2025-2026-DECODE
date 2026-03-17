package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.AutonomousPathing;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.BlueClosePathing;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;

/**
 * This is the OpMode for the Blue Close autonomous routine. It initializes the robot, follower, and path,
 * and runs the autonomous actions in the loop. The robot will start at a specific pose, follow a series
 * of paths to score preloads and pick up additional game elements, and will also check for motif detection
 * to adjust its behavior accordingly.
 */
@Autonomous(name = "Blue Close", group = "Active")
public class BlueClose extends AutonomousOpMode {
    /**
     * This method returns the autonomous path to be followed during this routine.
     *
     * @return an instance of the BlueCloseAutonomous class.
     */
    @Override
    protected AutonomousPathing getPath() {
        return new BlueClosePathing(drive.getFollower(), robot.intake, robot.shooter, robot.transfer);
    }

    /**
     * This method returns the starting pose for the robot.
     *
     * @return the starting pose of the robot for this autonomous routine.
     */
    @Override
    protected Pose getStartingPose() {
        return BlueClosePathing.startPose;
    }

    /**
     * This method returns the team's alliance.
     *
     * @return the base for this autonomous routine, which is used to set the turret base values for
     * the shooter.
     */
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    /**
     * This method switches the Limelight camera to the pipeline used to detect the blue goal.
     */
    @Override
    protected void switchToShootingPipeline() {
        robot.limelight.switchToBlueGoal();
    }

}
