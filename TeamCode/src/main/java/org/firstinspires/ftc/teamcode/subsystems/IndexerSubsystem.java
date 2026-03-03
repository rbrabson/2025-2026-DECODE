package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;
import org.firstinspires.ftc.teamcode.util.PatternUtil;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/**
 * Manages the indexer servo position and ball color detection during intake.
 *
 * This subsystem handles two responsibilities:
 *
 * 1. INDEXER POSITIONING FOR INTAKE:
 *    When the robot is picking up balls, the indexer needs to rotate so the
 *    next empty slot is aligned with the intake. This subsystem reads the
 *    pattern string to find the next 'X' (empty slot) and positions the
 *    indexer accordingly.
 *
 * 2. COLOR DETECTION:
 *    As each ball enters the indexer, the webcam detects whether it's green
 *    or purple. The pattern string is updated to record the color.
 *    A 500ms debounce timer prevents double-detection (the same ball being
 *    counted twice).
 *
 * 3. CENTER CONTROL (pre-positioning for shooting):
 *    When all 3 slots are full, the indexer pre-positions to the correct
 *    first-shot position based on motif alignment. This saves time when
 *    the shooting sequence begins.
 *
 * TODO: [Student] What would happen if the color sensor misidentifies a ball?
 *       For example, if a green ball is detected as purple. How would that
 *       affect the shooting order?
 *
 * TODO: [Student] Why do we need a 500ms debounce timer? What would happen
 *       if we detected colors every single loop iteration?
 */
public class IndexerSubsystem {

    private final Servo indexer;
    private final PredominantColorProcessor colorSensor;
    private final IndexerConfig config;

    /** Debounce timer to prevent double-detection of the same ball. */
    private final ElapsedTime colorDebounce = new ElapsedTime();

    /** Timer for center control delay. */
    private final ElapsedTime centerTimer = new ElapsedTime();

    /**
     * The current pattern string representing what's in each indexer slot.
     * 'G' = green, 'P' = purple, 'X' = empty.
     */
    private String pattern;

    /**
     * Whether the indexer has been pre-positioned for the first shot.
     * Once all 3 balls are loaded and the motif is known, we move the
     * indexer to the correct first-shot position immediately (before
     * the shooting sequence starts) to save time.
     */
    private boolean centerControl = false;

    /** Index of the current empty slot (0, 1, 2, or -1 if all full). */
    private int currentEmptySlot = 0;

    /**
     * Creates an IndexerSubsystem.
     *
     * @param indexer         the indexer servo from RobotHardware
     * @param colorSensor     the color sensor processor from RobotHardware
     * @param config          the indexer position configuration
     * @param initialPattern  starting pattern (e.g., "GPP" for auto, "XXX" for teleop)
     */
    public IndexerSubsystem(Servo indexer, PredominantColorProcessor colorSensor,
                            IndexerConfig config, String initialPattern) {
        this.indexer = indexer;
        this.colorSensor = colorSensor;
        this.config = config;
        this.pattern = initialPattern;
    }

    /**
     * Updates the indexer position and checks for new ball colors.
     * Must be called every loop iteration.
     *
     * @param isShooting   true if the ShootingStateMachine is currently active
     * @param isShooting2  true if manual shooting is active (TeleOp only)
     * @param motif        the detected motif string (e.g., "GPP")
     * @param telemetry    for debug output
     */
    public void update(boolean isShooting, boolean isShooting2, String motif,
                       Telemetry telemetry) {
        char x1 = 'X';
        char green = 'G';
        char purple = 'P';

        // -- CENTER CONTROL: Pre-position for shooting when all balls are loaded --
        if (PatternUtil.count(pattern, x1) == 0 && !centerControl && !motif.isEmpty()) {
            positionForFirstShot(motif);
            centerControl = true;
            centerTimer.reset();
            return;
        }

        // -- INTAKE MODE: Position indexer for next empty slot and detect colors --
        currentEmptySlot = PatternUtil.getNextEmptySlot(pattern);

        if (!isShooting && !isShooting2 && !centerControl && currentEmptySlot != -1) {
            // Move indexer to align empty slot with intake
            indexer.setPosition(config.getIntakePosition(currentEmptySlot));

            // Check color sensor for a new ball
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            if (PatternUtil.hasEmptySlots(pattern)
                && !isShooting && !isShooting2 && !centerControl) {

                if (colorDebounce.milliseconds() > 500) {
                    if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN) {
                        pattern = PatternUtil.replaceAt(pattern, currentEmptySlot, 'G');
                        colorDebounce.reset();
                    } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE) {
                        pattern = PatternUtil.replaceAt(pattern, currentEmptySlot, 'P');
                        colorDebounce.reset();
                    }
                }
            }

            telemetry.addData("Color Detected", result.closestSwatch);
        }

        telemetry.addData("Pattern", pattern);
    }

    /**
     * Pre-positions the indexer for the first shot based on motif alignment.
     *
     * This is called when all 3 balls have been loaded and the motif is known.
     * By moving the indexer now (before shooting starts), we save time.
     *
     * The logic mirrors calculateShootOrder() in ShootingStateMachine:
     *   - If green positions align: start at slot 1
     *   - If offset by +1: start at slot 3
     *   - If offset by +2: start at slot 2
     *
     * TODO: [Student] Trace through this logic with an example.
     *       If pattern = "PGP" and motif = "GPP", which position does the
     *       indexer move to? Why?
     *
     * @param motif  the detected motif from AprilTag
     */
    private void positionForFirstShot(String motif) {
        char green = 'G';
        char purple = 'P';

        if (PatternUtil.count(pattern, green) == 1
            && PatternUtil.count(pattern, purple) == 2) {

            int motifGreenPos = motif.indexOf(green);
            int patternGreenPos = pattern.indexOf(green);

            if (motifGreenPos == patternGreenPos) {
                indexer.setPosition(config.pos1Shoot);
            } else if (motifGreenPos == (patternGreenPos + 1) % 3) {
                indexer.setPosition(config.pos3Shoot);
            } else {
                indexer.setPosition(config.pos2Shoot);
            }
        } else {
            // Non-standard pattern (e.g., 3 of the same color): default to slot 1
            indexer.setPosition(config.pos1Shoot);
        }
    }

    // -----------------------------------------------------------------------
    // Getters and setters
    // -----------------------------------------------------------------------

    /** Returns the current pattern string. */
    public String getPattern() {
        return pattern;
    }

    /** Sets the pattern string (used when shooting resets it to "XXX"). */
    public void setPattern(String pattern) {
        this.pattern = pattern;
    }

    /** Returns whether the indexer has been pre-positioned for shooting. */
    public boolean isCenterControl() {
        return centerControl;
    }

    /** Sets the center control flag. */
    public void setCenterControl(boolean centerControl) {
        this.centerControl = centerControl;
    }

    /** Resets the subsystem for a new intake cycle. */
    public void resetForNewCycle() {
        pattern = "XXX";
        centerControl = false;
        colorDebounce.reset();
    }
}
