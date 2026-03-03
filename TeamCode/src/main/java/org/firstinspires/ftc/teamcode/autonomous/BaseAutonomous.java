package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.AllianceConfig;
import org.firstinspires.ftc.teamcode.util.SharedState;

/**
 * Abstract base class for all autonomous OpModes.
 *
 * This class handles all the shared boilerplate that was previously
 * copy-pasted into every autonomous file:
 *   - Hardware initialization (via RobotHardware)
 *   - Subsystem creation and wiring
 *   - Main loop orchestration (follower update, turret tracking, shooting, intake, indexer)
 *   - Telemetry output
 *   - SharedState updates for auto-to-teleop handoff
 *
 * HOW TO CREATE A NEW AUTONOMOUS:
 * Extend this class and implement the abstract methods:
 *
 *   1. getStartPose()           - Where does the robot start on the field?
 *   2. getAlliance()             - Which alliance? (BLUE or RED)
 *   3. getIndexerConfig()       - Which indexer servo config? (CONFIG_A or CONFIG_B)
 *   4. getShootingDelays()      - Timing for the first shot {indexer, hinge, close}
 *   5. getInitialPattern()      - What pattern to start with (usually "GPP")
 *   6. buildPaths(Follower)     - Define all the path chains
 *   7. autonomousPathUpdate()   - The state machine that sequences paths and shooting
 *
 * TODO: [Student] Look at BlueFarAuto.java as an example of a concrete autonomous.
 *       Notice how short it is compared to the original JWBF.java. All the
 *       shared logic lives here in BaseAutonomous instead of being duplicated.
 */
public abstract class BaseAutonomous extends OpMode {

    // -----------------------------------------------------------------------
    // Shared infrastructure (available to all subclasses)
    // -----------------------------------------------------------------------

    /** PedroPathing follower for path following and odometry. */
    protected Follower follower;

    /** Timer that resets whenever the path state changes. */
    protected Timer pathTimer;

    /** Timer tracking total opmode runtime. */
    protected Timer opmodeTimer;

    /** The current state in the path state machine. */
    protected int pathState;

    /** Central hardware initialization. */
    protected RobotHardware hw;

    /** Intake motor subsystem. */
    protected IntakeSubsystem intake;

    /** Turret tracking subsystem. */
    protected TurretSubsystem turret;

    /** Shooting state machine. */
    protected ShootingStateMachine shooter;

    /** Indexer positioning and color detection subsystem. */
    protected IndexerSubsystem indexerSub;

    // -----------------------------------------------------------------------
    // Abstract methods (must be implemented by each concrete autonomous)
    // -----------------------------------------------------------------------

    /**
     * Returns the starting pose of the robot on the field.
     * This is where the robot is physically placed before the match starts.
     *
     * @return the starting Pose (x, y in inches; heading in radians)
     */
    protected abstract Pose getStartPose();

    /**
     * Returns which alliance this autonomous is for.
     * Determines the turret target coordinates.
     *
     * @return AllianceConfig.BLUE or AllianceConfig.RED
     */
    protected abstract AllianceConfig getAlliance();

    /**
     * Returns which indexer servo configuration to use.
     *
     * @return IndexerConfig.CONFIG_A or IndexerConfig.CONFIG_B
     */
    protected abstract IndexerConfig getIndexerConfig();

    /**
     * Returns the timing delays for the first shot (in milliseconds).
     * Format: {indexerDelay, hingeDelay, closeDelay}
     *
     * @return array of 3 doubles representing timing delays
     */
    protected abstract double[] getShootingDelays();

    /**
     * Returns the initial ball pattern for the indexer.
     * Usually "GPP" (preloaded with 1 green + 2 purple) for autonomous.
     *
     * @return the initial pattern string
     */
    protected abstract String getInitialPattern();

    /**
     * Builds all path chains using the follower's path builder.
     * Called once during init().
     *
     * @param follower  the PedroPathing follower to build paths with
     */
    protected abstract void buildPaths(Follower follower);

    /**
     * The autonomous path state machine.
     * Called every loop iteration. Uses switch(pathState) to sequence
     * path following and shooting actions.
     *
     * Use setPathState(n) to advance to the next state.
     * Use triggerShoot() to start a shooting sequence.
     * Use follower.followPath(path) to start following a path.
     * Use !follower.isBusy() to check if a path is complete.
     * Use !shooter.isShooting() to check if shooting is complete.
     */
    protected abstract void autonomousPathUpdate();

    // -----------------------------------------------------------------------
    // OpMode lifecycle methods
    // -----------------------------------------------------------------------

    /**
     * Called once when the OpMode is initialized (INIT button pressed).
     * Sets up all hardware, subsystems, and paths.
     */
    @Override
    public void init() {
        // Create timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize hardware
        hw = new RobotHardware(hardwareMap);
        hw.init();

        // Create PedroPathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());

        // Create subsystems
        AllianceConfig alliance = getAlliance();
        IndexerConfig idxConfig = getIndexerConfig();
        double[] delays = getShootingDelays();

        intake = new IntakeSubsystem(hw.intake);
        turret = new TurretSubsystem(hw.turret, hw.limelight,
            alliance.targetX, alliance.targetY);
        shooter = new ShootingStateMachine(hw.indexer, hw.hinge, idxConfig,
            delays[0], delays[1], delays[2]);
        indexerSub = new IndexerSubsystem(hw.indexer, hw.colorSensor,
            idxConfig, getInitialPattern());

        // Build paths
        buildPaths(follower);

        // Configure telemetry for fast updates
        telemetry.setMsTransmissionInterval(10);
    }

    /**
     * Called continuously after init while waiting for START.
     * Override in subclass if you need init_loop behavior.
     */
    @Override
    public void init_loop() {
        // Subclasses can override if needed
    }

    /**
     * Called once when the START button is pressed.
     * Activates PedroPathing's PID controllers and starts the state machine.
     */
    @Override
    public void start() {
        follower.activateAllPIDFs();
        setPathState(0);
    }

    /**
     * Called repeatedly after START. This is the main autonomous loop.
     *
     * Orchestration order:
     *   1. Set hood position
     *   2. Update PedroPathing follower (odometry + path following)
     *   3. Run the path state machine (subclass logic)
     *   4. Update turret tracking
     *   5. Run shooter at constant velocity
     *   6. Update shooting state machine (if active)
     *   7. Run intake
     *   8. Update indexer (color detection + positioning)
     *   9. Sync pattern from shooting state machine
     *  10. Update SharedState for teleop handoff
     *  11. Output telemetry
     *
     * TODO: [Student] Why does the order matter? What would happen if we
     *       updated the turret BEFORE the follower? Would the turret angle
     *       calculation use stale position data?
     */
    @Override
    public void loop() {
        // 1. Set hood to default position
        hw.hoodExtension.setPosition(0);

        // 2. Update path following and odometry
        follower.update();

        // 3. Run the subclass's path state machine
        autonomousPathUpdate();

        // 4. Update turret tracking (aims at basket)
        turret.update(
            follower.getPose().getX(),
            follower.getPose().getY(),
            follower.getHeading(),
            telemetry, gamepad1, gamepad2
        );

        // 5. Keep shooter flywheel spinning at constant velocity
        hw.shooter.setVelocity(RobotHardware.SHOOTER_VELOCITY);

        // 6. Update shooting state machine
        if (shooter.isShooting()) {
            shooter.update(telemetry);
        }

        // 7. Keep intake running
        intake.run(true);

        // 8. Update indexer (color detection + positioning)
        indexerSub.update(shooter.isShooting(), false, turret.getMotif(), telemetry);

        // 9. Sync pattern: when shooting finishes, it resets pattern to "XXX"
        if (!shooter.isShooting() && "XXX".equals(shooter.getPattern())) {
            indexerSub.setPattern("XXX");
            indexerSub.setCenterControl(false);
        }

        // 10. Update SharedState for auto-to-teleop handoff
        SharedState.xPos = follower.getPose().getX();
        SharedState.yPos = follower.getPose().getY();
        SharedState.yaw = follower.getPose().getHeading();
        SharedState.motif = turret.getMotif();
        SharedState.turretPose = hw.turret.getCurrentPosition();

        // 11. Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Pattern", indexerSub.getPattern());
        telemetry.addData("Motif", turret.getMotif());
        telemetry.update();
    }

    /**
     * Called when the OpMode is stopped.
     * FTC SDK automatically stops all motors/servos.
     */
    @Override
    public void stop() {
        // Everything auto-disables via FTC SDK
    }

    // -----------------------------------------------------------------------
    // Helper methods for subclasses
    // -----------------------------------------------------------------------

    /**
     * Advances the path state machine to a new state.
     * Resets the path timer so subclasses can use time-based transitions.
     *
     * @param newState  the new state number (-1 = finished/idle)
     */
    protected void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    /**
     * Triggers the shooting state machine to begin a 3-ball shooting sequence.
     *
     * Call this from autonomousPathUpdate() when the robot has reached a
     * scoring position and is ready to shoot.
     *
     * After calling this, check !shooter.isShooting() in the next state
     * to know when shooting is complete.
     */
    protected void triggerShoot() {
        shooter.startShooting(
            indexerSub.getPattern(),
            turret.getMotif(),
            indexerSub.isCenterControl()
        );
    }
}
