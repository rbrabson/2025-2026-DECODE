package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.PatternUtil;

/**
 * Manages the 3-ball shooting sequence as a clean state machine.
 *
 * This replaces the ~350-line nested if/else block that was previously
 * copy-pasted into every autonomous and teleop OpMode.
 *
 * HOW SHOOTING WORKS (step by step for each ball):
 *
 *   1. MOVE_INDEXER:   Rotate the indexer servo to align the correct
 *                      ball slot with the shooter flywheel.
 *
 *   2. WAIT_INDEXER:   Wait for the servo to physically reach its position.
 *                      First shot waits longer than subsequent shots.
 *
 *   3. OPEN_HINGE:     Open the hinge servo (position 0.4) to flick the
 *                      ball into the spinning flywheel.
 *
 *   4. WAIT_HINGE:     Wait for the ball to actually launch.
 *
 *   5. CLOSE_HINGE:    Close the hinge servo (position 0.09) back to its
 *                      resting position.
 *
 *   6. WAIT_CLOSE:     Wait for the hinge to fully close before moving
 *                      to the next ball.
 *
 *   7. Repeat steps 1-6 for all 3 balls.
 *
 * WHAT IS MOTIF ALIGNMENT?
 * The motif (detected from AprilTags) tells us the correct ORDER to shoot
 * the balls. If the motif is "GPP" (green, purple, purple), the green ball
 * needs to land in position 1 of the basket. The pattern string tells us
 * which slot of the indexer actually holds the green ball. By comparing
 * the motif and pattern, we figure out which slot to shoot first.
 *
 * TODO: [Student] Walk through an example:
 *       If motif = "PGP" and pattern = "GPP", which slot should be shot first?
 *       Trace through the calculateShootOrder() method to verify.
 *
 * TODO: [Student] Why does the first shot need a longer delay than subsequent
 *       shots? What is the indexer doing during that extra time?
 *
 * STATE MACHINE DIAGRAM:
 *
 *   IDLE --[startShooting()]--> MOVE_INDEXER --> WAIT_INDEXER --> OPEN_HINGE
 *     ^                                                              |
 *     |                                                              v
 *   FINISHED <--[3 balls done]-- WAIT_CLOSE <-- CLOSE_HINGE <-- WAIT_HINGE
 *     ^                              |
 *     |                              v
 *     +----[more balls]-----> MOVE_INDEXER (next ball)
 */
public class ShootingStateMachine {

    // -----------------------------------------------------------------------
    // State enum - each value represents one step in the shooting process
    // -----------------------------------------------------------------------

    /**
     * The possible states of the shooting state machine.
     *
     * TODO: [Student] Compare this to the old code. The old code used
     *       "iteration" and "flag" variables with deeply nested if/else.
     *       Which approach is easier to understand and debug? Why?
     */
    public enum State {
        /** Not shooting. Waiting for startShooting() to be called. */
        IDLE,
        /** Moving the indexer servo to the next ball's shoot position. */
        MOVE_INDEXER,
        /** Waiting for the indexer servo to reach its target position. */
        WAIT_INDEXER,
        /** Opening the hinge to flick the ball into the flywheel. */
        OPEN_HINGE,
        /** Waiting for the hinge to fully open and the ball to launch. */
        WAIT_HINGE,
        /** Closing the hinge back to its resting position. */
        CLOSE_HINGE,
        /** Waiting for the hinge to fully close. */
        WAIT_CLOSE,
        /** All 3 balls have been shot. Cleaning up. */
        FINISHED
    }

    // -----------------------------------------------------------------------
    // Hardware references
    // -----------------------------------------------------------------------
    private final Servo indexer;
    private final Servo hinge;
    private final IndexerConfig config;

    // -----------------------------------------------------------------------
    // State tracking
    // -----------------------------------------------------------------------
    private State state = State.IDLE;
    private int currentIteration = 0;    // Which ball we're on (0, 1, or 2)
    private double[] shootOrder;          // The 3 indexer positions to shoot in sequence
    private boolean isCenterControl;

    // -----------------------------------------------------------------------
    // Timers
    // -----------------------------------------------------------------------
    private final ElapsedTime timer = new ElapsedTime();

    // -----------------------------------------------------------------------
    // Timing configuration (milliseconds)
    // These differ between robot configs:
    //   JWBF uses 500/675/850 (faster first shot)
    //   JWBF_Finish/JWRC/JWBC/JWRF use 1000/1175/1350 (slower, more reliable)
    // -----------------------------------------------------------------------
    private final double firstShotIndexerDelay;
    private final double firstShotHingeDelay;
    private final double firstShotCloseDelay;

    /** Delay before subsequent (non-first) shots: indexer settling time. */
    private static final double SUBSEQUENT_INDEXER_DELAY = 150;
    /** Delay for hinge open on subsequent shots. */
    private static final double SUBSEQUENT_HINGE_DELAY = 325;
    /** Delay for hinge close on subsequent shots. */
    private static final double SUBSEQUENT_CLOSE_DELAY = 500;

    /** Delay for hinge open when center-controlled (indexer already positioned). */
    private static final double CENTER_HINGE_DELAY = 175;
    /** Delay for hinge close when center-controlled. */
    private static final double CENTER_CLOSE_DELAY = 350;

    // -----------------------------------------------------------------------
    // Output state (read by other subsystems)
    // -----------------------------------------------------------------------
    private boolean shooting = false;
    private String pattern = "XXX";
    private boolean centerControl = false;

    /**
     * Creates a ShootingStateMachine.
     *
     * @param indexer               the indexer servo from RobotHardware
     * @param hinge                 the hinge servo from RobotHardware
     * @param config                the indexer position configuration
     * @param firstShotIndexerDelay ms to wait before first shot's hinge open
     * @param firstShotHingeDelay   ms for first shot's hinge to stay open
     * @param firstShotCloseDelay   ms for first shot's hinge to close
     */
    public ShootingStateMachine(Servo indexer, Servo hinge, IndexerConfig config,
                                double firstShotIndexerDelay,
                                double firstShotHingeDelay,
                                double firstShotCloseDelay) {
        this.indexer = indexer;
        this.hinge = hinge;
        this.config = config;
        this.firstShotIndexerDelay = firstShotIndexerDelay;
        this.firstShotHingeDelay = firstShotHingeDelay;
        this.firstShotCloseDelay = firstShotCloseDelay;
    }

    /**
     * Starts a new 3-ball shooting sequence.
     *
     * @param pattern         current ball pattern (e.g., "GPP" or "PPG")
     * @param motif           detected motif from AprilTag (e.g., "GPP")
     * @param isCenterControl whether the indexer is already pre-positioned
     *                        for the first shot (skips indexer delay)
     */
    public void startShooting(String pattern, String motif, boolean isCenterControl) {
        this.pattern = pattern;
        this.isCenterControl = isCenterControl;
        this.currentIteration = 0;
        this.shooting = true;

        // Figure out which slots to shoot in which order based on motif alignment
        this.shootOrder = calculateShootOrder(pattern, motif);

        // Start the state machine
        this.state = State.MOVE_INDEXER;
        timer.reset();
    }

    /**
     * Must be called every loop iteration. Advances the state machine.
     *
     * This is the heart of the shooting logic. Each call checks the current
     * state, waits for timers, and transitions to the next state when ready.
     *
     * @param telemetry  for debug output to Driver Station
     */
    public void update(Telemetry telemetry) {
        if (!shooting) return;

        // Determine which timing values to use for this iteration
        boolean isFirstShot = (currentIteration == 0);
        boolean useCenterTiming = isFirstShot && isCenterControl;

        switch (state) {
            case MOVE_INDEXER:
                // Move indexer to the correct slot for this ball
                if (!useCenterTiming) {
                    indexer.setPosition(shootOrder[currentIteration]);
                }
                // Else: indexer already positioned by centerControl logic
                timer.reset();
                state = State.WAIT_INDEXER;
                break;

            case WAIT_INDEXER:
                // Wait for indexer to physically reach position
                double indexerDelay;
                if (useCenterTiming) {
                    indexerDelay = 0; // Already positioned
                } else if (isFirstShot) {
                    indexerDelay = firstShotIndexerDelay; // Longer wait for first shot
                } else {
                    indexerDelay = SUBSEQUENT_INDEXER_DELAY; // Short wait for later shots
                }

                if (timer.milliseconds() > indexerDelay) {
                    // Open the hinge to flick the ball
                    hinge.setPosition(RobotHardware.HINGE_OPEN);
                    timer.reset();
                    state = State.WAIT_HINGE;
                }
                break;

            case WAIT_HINGE:
                // Wait for the hinge to open and the ball to launch
                double hingeDelay;
                if (useCenterTiming) {
                    hingeDelay = CENTER_HINGE_DELAY;
                } else if (isFirstShot) {
                    hingeDelay = firstShotHingeDelay;
                } else {
                    hingeDelay = SUBSEQUENT_HINGE_DELAY;
                }

                if (timer.milliseconds() > hingeDelay) {
                    // Close the hinge
                    hinge.setPosition(RobotHardware.HINGE_CLOSED);
                    timer.reset();
                    state = State.WAIT_CLOSE;
                }
                break;

            case WAIT_CLOSE:
                // Wait for the hinge to close fully before next ball
                double closeDelay;
                if (useCenterTiming) {
                    closeDelay = CENTER_CLOSE_DELAY;
                } else if (isFirstShot) {
                    closeDelay = firstShotCloseDelay;
                } else {
                    closeDelay = SUBSEQUENT_CLOSE_DELAY;
                }

                if (timer.milliseconds() > closeDelay) {
                    currentIteration++;
                    if (currentIteration >= 3) {
                        // All 3 balls shot!
                        state = State.FINISHED;
                    } else {
                        // Move to next ball
                        timer.reset();
                        state = State.MOVE_INDEXER;
                    }
                }
                break;

            case FINISHED:
                // Clean up and return to idle
                shooting = false;
                pattern = "XXX";
                centerControl = false;
                state = State.IDLE;
                break;

            default:
                break;
        }

        // Debug telemetry
        telemetry.addData("Shoot State", state);
        telemetry.addData("Shoot Ball #", currentIteration + 1);
    }

    /**
     * Calculates the order of indexer positions to shoot in based on
     * motif-pattern alignment.
     *
     * THE ALGORITHM:
     * We have 3 balls in the indexer (described by "pattern", e.g., "GPP").
     * The motif tells us which POSITION in the basket the green ball should go.
     * We need to figure out which SLOT to shoot first so each ball lands
     * in the correct basket position.
     *
     * Case 1: Green positions match (motifDetect == patternDetect)
     *   Shoot in order: slot 1, slot 2, slot 3
     *
     * Case 2: Green is offset by +1 (motifDetect == (patternDetect+1)%3)
     *   Start from slot 3, then shift up by increment each time
     *
     * Case 3: Green is offset by +2
     *   Start from slot 2, then shift up by increment each time
     *
     * TODO: [Student] Work through each case with a concrete example.
     *       Example: motif = "PGP", pattern = "GPP"
     *       Which case does this fall into? What is shootOrder?
     *
     * @param pattern  the current ball pattern in the indexer
     * @param motif    the detected motif from AprilTag
     * @return array of 3 servo positions in the order they should be shot
     */
    private double[] calculateShootOrder(String pattern, String motif) {
        char green = 'G';
        char purple = 'P';

        // Check if we have a valid 1G+2P pattern with a detected motif
        if (PatternUtil.count(pattern, green) == 1
            && PatternUtil.count(pattern, purple) == 2
            && !motif.isEmpty()) {

            int motifGreenPos = motif.indexOf(green);
            int patternGreenPos = pattern.indexOf(green);

            if (motifGreenPos == patternGreenPos) {
                // Case 1: Green positions align - shoot slots in direct order
                return new double[]{
                    config.pos1Shoot,
                    config.pos2Shoot,
                    config.pos3Shoot
                };
            } else if (motifGreenPos == (patternGreenPos + 1) % 3) {
                // Case 2: Offset by +1 - start from slot 3 and increment
                return new double[]{
                    config.pos3Shoot,
                    config.pos3Shoot + config.increment,
                    config.pos3Shoot + (config.increment * 2)
                };
            } else {
                // Case 3: Offset by +2 - start from slot 2 and increment
                return new double[]{
                    config.pos2Shoot,
                    config.pos2Shoot + config.increment,
                    config.pos2Shoot + (config.increment * 2)
                };
            }
        } else {
            // No valid motif or non-standard pattern: shoot in default order
            return new double[]{
                config.pos1Shoot,
                config.pos2Shoot,
                config.pos3Shoot
            };
        }
    }

    // -----------------------------------------------------------------------
    // Getters and control
    // -----------------------------------------------------------------------

    /** Returns true if a shooting sequence is currently in progress. */
    public boolean isShooting() {
        return shooting;
    }

    /** Returns the current pattern string (becomes "XXX" after shooting completes). */
    public String getPattern() {
        return pattern;
    }

    /** Returns the current state of the state machine. */
    public State getState() {
        return state;
    }

    /**
     * Immediately stops the shooting sequence and resets to idle.
     * Use this for emergency stops or when transitioning between OpModes.
     */
    public void reset() {
        shooting = false;
        state = State.IDLE;
        currentIteration = 0;
        centerControl = false;
        timer.reset();
    }
}
