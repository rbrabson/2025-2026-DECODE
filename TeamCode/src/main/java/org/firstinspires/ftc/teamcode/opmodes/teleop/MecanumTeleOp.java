package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.inputprocessors.UserInputProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.IntakeProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.MecanumDriveProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.ShootArtifactProcessor;
import org.firstinspires.ftc.teamcode.inputprocessors.ShooterProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.Mechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroFollower;

import java.util.Arrays;
import java.util.List;

/**
 * This is the base OpMode for teleop routines. It provides the structure for initializing,
 * starting, and looping during the teleop period.
 */
public abstract class MecanumTeleOp extends OpMode {
    private static final Pose DEFAULT_STARTING_POSE = new Pose(7.5, 7.5, Math.toRadians(90));

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Timer timer = new Timer();
    protected Robot robot;
    private List<UserInputProcessor> inputHandlers;
    private List<Mechanism> mechanisms;

    private FusedLocalizer localizer;
    private Alliance alliance;

    /**
     * This method is called once when the driver hits the INIT button. It is used to initialize
     * any hardware or variables needed for the teleop period. In this base class, it is left empty
     * and can be overridden by subclasses to provide specific initialization logic.
     */
    @Override
    public void init() {
        Robot.reset();
        robot = Robot.getInstance(hardwareMap, telemetry);

        Pose startingPose = blackboard.containsKey("robotPose") ? (Pose) blackboard.get("robotPose") : DEFAULT_STARTING_POSE;
        localizer = PedroFollower.getFusedLocalizer(hardwareMap, robot.limelight.getSensor()).withMode(FusedLocalizer.Mode.TELEOP);
        localizer.setStartPose(startingPose);
        robot.mecanumDrive.setLocalizer(localizer);
        alliance = getAlliance();
        robot.shooter.setTurretBaseValues(alliance.getBaseX(), alliance.getBaseY());

        // Set all Lynx module hubs to manual bulk caching mode. This allows us to control when
        // the bulk cache is cleared, which can improve performance by reducing the number of reads
        // performed during the loop.
        for (LynxModule hub : robot.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize the input handlers for each mechanism.
        inputHandlers = Arrays.asList(
                new MecanumDriveProcessor(robot.mecanumDrive, telemetry),
                new IntakeProcessor(robot.intake, telemetry),
                new ShooterProcessor(robot.shooter, robot.limelight, localizer, alliance, telemetry),
                new ShootArtifactProcessor(robot.intake, robot.transfer, robot.shooter, telemetry)
        );

        mechanisms = Arrays.asList(
                robot.mecanumDrive,
                robot.intake,
                robot.shooter,
                robot.transfer
        );

        telemetry.addLine("Initialization Complete");
    }

    /**
     * This method must be implemented by subclasses to specify the alliance (RED or BLUE) that
     * the robot is on.
     *
     * @return The alliance that the robot is on (RED or BLUE).
     */
    abstract protected Alliance getAlliance();

    /**
     * This method is called once when the driver hits the PLAY button. It is used to start any
     * processes or set any initial conditions needed for the teleop period. In this base class, it
     * is left empty and can be overridden by subclasses to provide specific starting logic.
     */
    @Override
    public void start() {
        robot.shooter.setFlywheelRPMToLow();

        // Warm-up update to initialize follower state before loop begins
        localizer.update();
    }

    /**
     * This method is called repeatedly during the teleop period. It is where the main logic for
     * controlling the robot during teleop should be placed. In this base class, it is left empty
     * and can be overridden by subclasses to provide specific control logic.
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

        // Update the robot's pose
        localizer.update();

        // Adjust the shooter's flywheel velocity, hood position, and turret angle based on the
        // robot's current location
        robot.shooter.update(localizer, alliance);

        // Update any hardware components on each loop
        for (Mechanism mechanism : mechanisms) {
            mechanism.update();
        }

        // Update each input handler with the current gamepad values. Each handler will handle
        // the logic for controlling its respective mechanism based on the gamepad inputs.
        for (UserInputProcessor controller : inputHandlers) {
            controller.process(currentGamepad1, currentGamepad2);
        }

        telemetry.addData("Pose", localizer.getPose());
        telemetry.addData("Loop Time (s)", "%.2f", timer.getElapsedTimeSeconds());

        telemetry.update();
    }

    /**
     * This method is called once when the driver hits the STOP button. It is used to stop any
     * processes or perform any cleanup needed after the teleop period.
     */
    @Override
    public void stop() {
        robot.intake.close();
    }
}
