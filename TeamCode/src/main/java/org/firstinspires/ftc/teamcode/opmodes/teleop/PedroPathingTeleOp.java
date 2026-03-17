package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.inputprocessors.IntakeProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.PedroPathingProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.ShootArtifactProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.ShooterProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.UserInputProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PedroPathingDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedLocalizer;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * This class implements a teleop mode for the robot that utilizes the Pedro Pathing library for
 * localization and path following. The robot's starting pose is determined by checking if a pose
 * was stored on the blackboard during the autonomous period; if not, it defaults to a predefined
 * starting pose. The teleop mode includes input processors for controlling the robot's mechanisms
 * based on gamepad inputs, and it updates the robot's state and telemetry on each loop.
 */
public abstract class PedroPathingTeleOp extends OpMode {
    private static final Pose DEFAULT_STARTING_POSE = new Pose(7.5, 7.5, Math.toRadians(90));

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Timer timer = new Timer();
    private Robot robot;
    private List<UserInputProcessor> inputHandlers;
    private List<Mechanism> mechanisms;

    private PedroPathingDrive drive;
    private Alliance alliance;

    /**
     * The init method is called once when the driver hits the "Init" button on the driver station.
     * It initializes the robot hardware, sets up the localizer with the starting pose, configures
     * the input handlers for controlling the robot's mechanisms, and prepares the telemetry.
     */
    @Override
    public void init() {
        Robot.reset();
        robot = Robot.getInstance(hardwareMap, telemetry);

        Pose startingPose = Objects.requireNonNull(blackboard.containsKey("robotPose") ? (Pose) blackboard.get("robotPose") : DEFAULT_STARTING_POSE);
        drive = robot.pedroPathingDrive
                    .setRobotCentric(false)
                    .setUseCompensation(true)
                    .setUseVoltageCompensation(true)
                    .setMode(FusedLocalizer.Mode.TELEOP)
                    .setStartingPose(startingPose);

        alliance = getAlliance();
        robot.shooter = robot.shooter
                .setTurretBaseValues(alliance.getBaseX(), alliance.getBaseY())
                .setAllianceAndPose(alliance, startingPose);

        // Set all Lynx module hubs to manual bulk caching mode. This allows us to control when
        // the bulk cache is cleared, which can improve performance by reducing the number of reads
        // performed during the loop.
        for (LynxModule hub : robot.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize the input handlers for each mechanism.
        inputHandlers = Arrays.asList(
                new PedroPathingProcessor(drive, telemetry),
                new IntakeProcessor(robot.intake, telemetry),
                new ShooterProcessor(robot.shooter, robot.limelight, drive.localizer, alliance, telemetry),
                new ShootArtifactProcessor(robot.intake, robot.transfer, robot.shooter, telemetry)
        );

        // All the hardware mechanisms that need to be updated each loop are added to this list.
        // This ensures that any necessary updates (such as setting motor powers or updating sensor
        // readings) are performed on each mechanism during the loop.
        mechanisms = Arrays.asList(
                robot.pedroPathingDrive,
                robot.intake,
                robot.shooter,
                robot.transfer
        );

        telemetry.addLine("Initialization Complete");

    }

    /**
     * The start method is called once when the driver hits the "Play" button on the driver station.
     */
    @Override
    public void start() {
        robot.shooter.setFlywheelRPMToLow();
        drive.startTeleopDrive();
        drive.update();
    }

    /**
     * The loop method is called repeatedly while the op mode is active.
     */
    @Override
    public void loop() {
        timer.resetTimer();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        // Always get a copy of the current gamepads. The gamepads are updated in near real time,
        // so this ensures that a consistent set of values is used within each mechanism.
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        drive.update();

        // Adjust the shooter's flywheel velocity, hood position, and turret angle based on the
        // robot's current location
        robot.shooter.update(drive.localizer, alliance);

        // Update any hardware components on each loop
        for (Mechanism mechanism : mechanisms) {
            mechanism.update();
        }

        // Update each input processor with the current gamepad values. Each processor will handle
        // the logic for controlling its respective mechanism based on the gamepad inputs.
        for (UserInputProcessor controller : inputHandlers) {
            controller.process(currentGamepad1, currentGamepad2);
        }

        telemetry.addData("Pose", drive.getPose());
        telemetry.addData("Loop Time (s)", "%.2f", timer.getElapsedTimeSeconds());

        telemetry.update();
    }

    /**
     * The stop method is called once when the driver hits the "Stop" button on the driver station.
     */
    @Override
    public void stop() {
        robot.intake.close();
    }

    /**
     * This method must be implemented by subclasses to specify the alliance (RED or BLUE) that
     * the robot is on.
     *
     * @return The alliance that the robot is on (RED or BLUE).
     */
    abstract protected Alliance getAlliance();
}
