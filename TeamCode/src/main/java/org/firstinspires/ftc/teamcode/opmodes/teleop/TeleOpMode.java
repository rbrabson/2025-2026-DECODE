package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.inputprocessors.DriveProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.IntakeProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.ShootArtifactProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.ShooterProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.UserInputProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PedroPathingDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroFollower;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * TeleOpMode is an abstract class that serves as the foundation for TeleOp OpModes. It
 * handles the initialization of the robot hardware, setting up the localizer, configuring input
 * handlers for controlling the robot's mechanisms, and managing the main loop for processing
 * user inputs and updating the robot's state.
 */
public abstract class TeleOpMode extends OpMode {
    private static final boolean USE_PEDRO_PATHING = true; // PedroPathing or private MecanumDrive logic for TeleOp
    private static final Pose DEFAULT_STARTING_POSE = new Pose(7.5, 7.5, Math.toRadians(90));

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Timer timer = new Timer();
    private Robot robot;
    private Drive drive;
    private List<UserInputProcessor> inputHandlers;
    private List<Mechanism> mechanisms;

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

        // Set all Lynx module hubs to manual bulk caching mode. This allows us to control when
        // the bulk cache is cleared, which can improve performance by reducing the number of reads
        // performed during the loop.
        for (LynxModule hub : robot.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = getDrive(startingPose);
        Alliance alliance = getAlliance();
        robot.shooter.setTurretBaseValues(alliance.getBaseX(), alliance.getBaseY())
                .setAlliancePose(alliance, startingPose)
                .setLocalizer(drive.getLocalizer());

        // Initialize the input handlers for each mechanism. This ensures the input handlers are
        // updated during the loop.
        inputHandlers = Arrays.asList(
                new DriveProcessor(drive, telemetry),
                new IntakeProcessor(robot.intake, telemetry),
                new ShooterProcessor(robot.shooter, robot.limelight, telemetry),
                new ShootArtifactProcessor(robot.intake, robot.transfer, robot.shooter, telemetry)
        );

        // All the hardware mechanisms that need to be updated each loop are added to this list.
        // This ensures that any necessary updates (such as setting motor powers or updating sensor
        // readings) are performed on each mechanism during the loop.
        mechanisms = Arrays.asList(
                drive,
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
        drive.update();
        robot.shooter.update();
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
     * Helper method to initialize the drive mechanism based on the specified starting pose.
     *
     * @param startingPose The initial pose of the robot on the field, including its position
     *                     (x, y) and heading (theta). This
     * @return An initialized Drive instance configured for TeleOp control, using either PedroPathing
     *         or a MecanumDrive implementation based on the USE_PEDRO_PATHING flag.
     */
    private Drive getDrive(Pose startingPose) {
        FusedLocalizer localizer = PedroFollower.getFusedLocalizer(hardwareMap, robot.limelight.getSensor());
        if (USE_PEDRO_PATHING) {
            PedroPathingDrive drive = new PedroPathingDrive(hardwareMap, localizer, telemetry)
                    .setMode(FusedLocalizer.Mode.TELEOP)
                    .setStartingPose(startingPose)
                    .setRobotCentric(false)
                    .setUseCompensation(true)
                    .setUseVoltageCompensation(true)
                    .startTeleopDrive();
            drive.activateAllPIDFs();
            return drive;
        } else {
            return new MecanumDrive(hardwareMap, localizer, telemetry)
                    .setMode(FusedLocalizer.Mode.TELEOP)
                    .setStartPose(startingPose);
        }
    }

    /**
     * This method must be implemented by subclasses to specify the alliance (RED or BLUE) that
     * the robot is on.
     *
     * @return The alliance that the robot is on (RED or BLUE).
     */
    abstract protected Alliance getAlliance();
}
