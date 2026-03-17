package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PedroPathingDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.AutonomousPathing;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroFollower;
import org.firstinspires.ftc.teamcode.decode.Motif;

import java.util.Arrays;
import java.util.List;

/**
 * This abstract class serves as a base for autonomous OpModes that follow a path using the PedroPathFollower.
 * It initializes the robot, follower, and path, and provides abstract methods for subclasses to specify
 * the flywheel velocity, starting pose, and turret base values. The loop method updates the follower and
 * path, checks for motif detection, and updates the blackboard with the robot's pose and turret position.
 */
public abstract class AutonomousOpMode extends OpMode {
    private FusedLocalizer localizer;
    protected Robot robot;
    private AutonomousPathing path;
    protected Follower follower;
    private final Timer timer = new Timer();
    private List<Mechanism> mechanisms;

    /**
     * This abstract method should be implemented by subclasses to specify the desired
     * AutonomousFollower for the autonomous routine. The AutonomousFollower represents the
     * specific path and actions that the robot will follow during the autonomous period. Each
     * subclass can implement this method to return a different AutonomousFollower that defines a
     * unique autonomous routine for the robot.
     *
     * @return The desired AutonomousFollower for the autonomous routine.
     */
    abstract protected AutonomousPathing getPath() ;

    /**
     * This abstract method should be implemented by subclasses to specify the base values for
     * the turret, which is used as a reference point for calculating the turret's target position
     * based on the target's coordinates. The base values are typically the coordinates of the
     * target (e.g., the goal).
     *
     * @return The base values for the turret, which are used as reference points for calculating
     *         the turret's target position based on the target's coordinates.
     */
    abstract protected Alliance getAlliance();

    /**
     * This abstract method should be implemented by subclasses to specify the starting pose of the
     * robot for the autonomous routine.
     *
     * @return The starting pose of the robot for the autonomous routine.
     */
    abstract protected Pose getStartingPose();

    /**
     * This abstract method should be implemented by subclasses to switch the Limelight camera to the
     * shooting pipeline. The pipeline is based on the alliance used for the OpMode.
     */
    abstract protected void switchToShootingPipeline();

    /**
     * This method is called once when the OpMode is initialized. It resets the robot, initializes
     * the robot and follower, and initializes the path with the follower and the robot's intake
     * and shooter. It also sets the turret base values for the shooter. This method prepares
     * everything for the autonomous period, but it does not start any actions yet. The actual
     * autonomous actions will be started in the start() method.
     */
    @Override
    public void init() {
        Robot.reset();
        robot = Robot.getInstance(hardwareMap, telemetry);

        localizer = PedroFollower.getFusedLocalizer(hardwareMap, robot.limelight.getSensor()).withMode(FusedLocalizer.Mode.AUTO);
        follower = PedroFollower.create(hardwareMap, localizer);
        follower.setStartingPose(getStartingPose());
        path = getPath();
        Alliance alliance = getAlliance();
        robot.shooter.setTurretBaseValues(alliance.getBaseX(), alliance.getBaseY());
        robot.shooter.setTurretBaseValues(alliance.getBaseX(), alliance.getBaseY())
                .setAlliancePose(alliance, getStartingPose())
                .setLocalizer(localizer);

        // Set all Lynx module hubs to manual bulk caching mode. This allows us to control when
        // the bulk cache is cleared, which can improve performance by reducing the number of reads
        // performed during the loop.
        for (LynxModule hub : robot.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        mechanisms = Arrays.asList(
                robot.intake,
                robot.shooter,
                robot.transfer
        );

        telemetry.addLine("Initialization Complete");
    }

    /**
     * This method is called once at the start of the OpMode. It runs all the setup actions,
     * including building paths and starting the path system. It sets the flywheel velocity, starts
     * the intake motor, enables the light, and activates all PIDF controllers for the follower. It
     * also updates the blackboard with default values for the robot's pose, turret position, and
     * motif detection to be preserved across OpModes.
     */
    @Override
    public void start() {
        robot.intake.startIntakeMotor();
        robot.light.enable();
        follower.activateAllPIDFs();
        robot.shooter.update();

        // Update the blackboard with default values to be preserved across OpModes
        blackboard.put("robotPose", localizer.getPose());
        blackboard.put("turretPosition", robot.shooter.getTurretCurrentPosition());
        blackboard.put("motif", Motif.getDefault());
    }

    /**
     * This method is called repeatedly while the OpMode is active. It updates the follower and
     * runs the path system. It also checks for motif detection and updates the motif variable accordingly
     */
    @Override
    public void loop() {
        timer.resetTimer();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        // Update any hardware components and the pathing on each loop
        for (Mechanism mechanism : mechanisms) {
            mechanism.update();
        }
        follower.update();
        path.update();

        if (!robot.limelight.isMotifDetected()) {
            Motif motif = robot.limelight.detectMotif();
            if (robot.limelight.isMotifDetected()) {
                blackboard.put("motif", motif);
                telemetry.addData("Motif Detected", motif);
                switchToShootingPipeline();
            }
        }

        blackboard.put("robotPose", follower.getPose());
        blackboard.put("turretPosition", robot.shooter.getTurretCurrentPosition());

        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Loop Time (s)", "%.2f", timer.getElapsedTimeSeconds());

        telemetry.update();
    }

    /**
     * This method is called once when the driver hits the "Stop" button on the driver station.
     * It stops the intake to perform any necessary cleanup actions.
     */
    @Override
    public void stop() {
        robot.intake.close();
    }
}
