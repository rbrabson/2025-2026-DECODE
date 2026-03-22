package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.Color;
import org.firstinspires.ftc.teamcode.decode.Motif;
import org.firstinspires.ftc.teamcode.sensors.ColorProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Class representing the indexer mechanism of the robot, responsible for
 * managing the game Artifacts.
 */
public class Indexer implements AutoCloseable{
    private static final double SHOOT_DELAY_TO_CENTER = 1000;
    private static final double SHOOT_DELAY_TO_NEXT_POS = 150;

    private static final double COLOR_DETECTION_TIMEOUT = 500;
    private static final double ROTATION_TIMEOUT = 750;

    private static final double POSITION_1_INTAKE = 0.6708;
    private static final double POSITION_2_INTAKE = 0.4695;
    private static final double POSITION_3_INTAKE = 0.2651;
    private static final double POSITION_1_SHOOT = 0.9829;
    private static final double POSITION_2_SHOOT = 0.7803;
    private static final double POSITION_3_SHOOT = 0.5706;

    private final Servo indexer;
    private final ColorProcessor colorProcessor;
    private final Telemetry telemetry;

    private final ElapsedTime timer = new ElapsedTime();
    private final Color[] artifactColors = new Color[3];
    private Position lastShootingPosition = null;
    private double timeWait = 0;
    private Motif motif = Motif.PPG; // Default motif, should be set when robot starts
    private State currentState = State.INACTIVE;

    private Position pendingPosition;
    private Position currentPosition;

    /**
     * Constructor for the Indexer class that initializes the indexer servo and color processor using
     * the provided hardware map.
     *
     * @param hardwareMap the hardware map to access the servo and color sensor
     */
    public Indexer(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for the Indexer class.
     *
     * @param hardwareMap the hardware map to access the servo and color sensor
     * @param telemetry   the telemetry object for logging information
     */
    public Indexer(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.indexer = map.get(Servo.class, "index");
        this.colorProcessor = new ColorProcessor(hardwareMap, telemetry);
        this.telemetry = Objects.requireNonNull(telemetry);

        Arrays.fill(artifactColors, null);

        currentPosition = Position.INTAKE_1; // Start at the first intake position

        telemetry.addLine("Indexer initialized");
    }

    /**
     * Sets the motif for the indexer, which may affect how it manages the game
     * Artifacts.
     *
     * @param motif the motif to set for the indexer
     */
    public void setMotif(@NonNull Motif motif) {
        this.motif = motif;
    }

    /**
     * Main method to handle the intake process of the indexer.
     *
     * @return the status of the intake process. This can be RUNNING if still in progress,
     *         SUCCESS if the artifact was successfully picked up, or FAILURE if there was an error
     *         during the intake process.
     */
    public Status intakeArtifact() {
        if (isFull()) {
            currentState = State.INACTIVE;
            return Status.FAILURE;
        }

        Status status;
        switch (currentState) {
            case INACTIVE:
                currentState = State.ROTATING;
                timeWait = ROTATION_TIMEOUT;
                timer.reset();
                // Fall through to the next case to start rotating immediately
            case ROTATING:
                status = rotateForIntake();
                if (status == Status.FAILURE) {
                    currentState = State.INACTIVE;
                    return Status.FAILURE;
                }
                if (status == Status.RUNNING) {
                    return Status.RUNNING;
                }
                // Rotation complete, start intaking
                currentState = State.INTAKING;
                timer.reset();
                return Status.RUNNING;
            case INTAKING:
                status = pickupArtifact();
                if (status == Status.FAILURE) {
                    currentState = State.INACTIVE;
                    return Status.FAILURE;
                }
                if (status == Status.RUNNING) {
                    return Status.RUNNING;
                }
                // Artifact picked up successfully
                currentState = State.INACTIVE;
                return Status.SUCCESS;
            default:
                currentState = State.INACTIVE;
                return Status.FAILURE;
        }
    }

    /**
     * Main method to handle the shooting process of the indexer.
     *
     * @return the status of the shooting process. This can be RUNNING if still in progress,
     *         SUCCESS if the artifact was successfully shot, or FAILURE if there was an error
     *         during the shooting process.
     */
    public Status shootArtifact() {
        if (isEmpty()) {
            currentState = State.INACTIVE;
            return Status.FAILURE;
        }

        Status status;
        switch (currentState) {
            case INACTIVE:
                currentState = State.ROTATING;
                timer.reset();
                // Fall through to the next case to start rotating immediately
            case ROTATING:
                status = rotateForShooting();
                if (status == Status.FAILURE) {
                    currentState = State.INACTIVE;
                    return Status.FAILURE;
                }
                if (status == Status.RUNNING) {
                    return Status.RUNNING;
                }
                // Rotation complete
                currentState = State.INACTIVE;
                return Status.SUCCESS;
            default:
                currentState = State.INACTIVE;
                return Status.FAILURE;
        }
    }

    /**
     * Checks if the indexer is full, meaning it has three artifacts in it.
     *
     * @return true if the indexer is full, false otherwise
     */
    public boolean isFull() {
        return !Arrays.asList(artifactColors).contains(null);
    }

    /**
     * Checks if the indexer is empty, meaning it has no artifacts in it.
     *
     * @return true if the indexer is empty, false otherwise
     */
    public boolean isEmpty() {
        for (Color color : artifactColors) {
            if (color != null) {
                return false;
            }
        }
        return true;
    }

    /**
     * Updates the indexer state to reflect that an artifact has been transferred from the intake
     * to the shooter. This method should be called after an artifact has been successfully
     * transferred to the shooter.
     */
    public Status transferComplete() {
        if (lastShootingPosition == null) {
            return Status.FAILURE; // No shooting position was set, so we can't update the indexer state
        }
        switch (lastShootingPosition) {
            case SHOOT_1:
                artifactColors[0] = null;
                break;
            case SHOOT_2:
                artifactColors[1] = null;
                break;
            case SHOOT_3:
                artifactColors[2] = null;
                break;
        }
        lastShootingPosition = null;
        return Status.SUCCESS;
    }

    // -------- Private helper methods --------

    /**
     * Wait for the indexer to rotate to the correct position for intaking an artifact.
     *
     * @return RUNNING if still waiting for rotation, SUCCESS if rotation is complete,
     *         or FAILURE if there was an error during rotation.
     */
    private Status rotateForIntake() {
        if (pendingPosition == null) {
            int empty = -1;
            for (int i =0; i < artifactColors.length; i++) {
                if (artifactColors[i] == null) { empty = i; break; }
            }
            if (empty <0) {
                return Status.FAILURE;
            }
            pendingPosition = Position.values()[empty]; // INTAKE_1..3 timeWait = ROTATION_TIMEOUT;
            setPosition(pendingPosition);
            timer.reset();
            return Status.RUNNING;
        }

        if (timer.milliseconds() < timeWait) {
            return Status.RUNNING;
        }
        currentPosition = pendingPosition;
        pendingPosition = null;
        return Status.SUCCESS;
    }

    /**
     * Uses the color sensor to detect the color of the artifact being picked up, and stores
     * that color in the artifactColors array at the index corresponding to the current intake
     * position.
     *
     * @return RUNNING if still waiting for color detection, SUCCESS if the artifact color was
     *         successfully detected and stored, or FAILURE if there was an error during color
     *         detection (such as a timeout)
     */
    private Status pickupArtifact() {
        if (timer.milliseconds() >= COLOR_DETECTION_TIMEOUT) return Status.FAILURE;

        Color detectedColor = colorProcessor.detectColor();
        if (detectedColor == null) return Status.RUNNING;

        int index = getIndexFromPosition(currentPosition);
        if (index <0 || index >= artifactColors.length) {
            return Status.FAILURE;
        }

        artifactColors[index] = detectedColor;
        return Status.SUCCESS;
    }

    /**
     * Helper method to get the index in the artifactColors array corresponding to a given position.
     *
     * @param position the position to get the index for (should be one of INTAKE_1, INTAKE_2,
     *                 or INTAKE_3)
     * @return the index in the artifactColors array corresponding to the given position,
     *         or -1 if the position is not a valid intake position
     */
    private int getIndexFromPosition(@NonNull Position position) {
        switch (position) {
            case INTAKE_1:
                return 0;
            case INTAKE_2:
                return 1;
            case INTAKE_3:
                return 2;
            default:
                return -1;
        }
    }

    /**
     * Rotates the indexer to the correct position for shooting an artifact based on the current
     * motif and the colors of the artifacts currently in the indexer.
     *
     * @return RUNNING if still waiting for rotation, SUCCESS if rotation is complete,
     *         or FAILURE if there was an error during rotation.
     */
    private Status rotateForShooting() {
        if (pendingPosition == null) {
            Color targetColor = getNextMotifColor();
            Position targetPosition = findShootingPosition(targetColor);
            if (targetPosition == null) return Status.FAILURE;

            pendingPosition = targetPosition;
            lastShootingPosition = targetPosition;
            timeWait = calculateShootingWaitTime(currentPosition);
            setPosition(targetPosition);
            timer.reset();
            return Status.RUNNING;
        }

        if (timer.milliseconds() < timeWait) return Status.RUNNING;
        currentPosition = pendingPosition;
        pendingPosition = null;
        return Status.SUCCESS;
    }

    /**
     * Determines the next color to shoot based on the current motif and the colors of the artifacts
     * currently in the indexer.
     *
     * @return the color of the next artifact to shoot based on the motif and current indexer state
     */
    private Color getNextMotifColor() {
        List<Color> motifColors = Arrays.asList(motif.colors());
        // Determine current index in motif based on which artifacts have been shot
        // This is a simplified version - adjust based on your actual tracking mechanism
        int shotCount = 0;
        for (Color color : artifactColors) {
            if (color == null) {
                shotCount++;
            }
        }
        return motifColors.get(shotCount % motifColors.size());
    }

    /**
     * Finds a shooting position for the next artifact to shoot based on the target color and the colors
     * of the artifacts currently in the indexer.
     *
     * @param targetColor the color of the artifact we want to shoot next based on the motif
     * @return the Position to shoot from that has the target color, or any filled position if
     * no match, or null if no artifacts are available
     */
    @Nullable
    private Position findShootingPosition(Color targetColor) {
        // First, try to find a position with matching color
        for (int i = 0; i < artifactColors.length; i++) {
            if (artifactColors[i] == targetColor) {
                return Position.values()[i + 3]; // +3 to get SHOOT_1, SHOOT_2, or SHOOT_3
            }
        }

        // If no match, find any filled position
        for (int i = 0; i < artifactColors.length; i++) {
            if (artifactColors[i] != null) {
                return Position.values()[i + 3];
            }
        }

        return null; // No artifacts available
    }

    /**
     * Calculates the wait time for the indexer to move from the current position to the target
     * shooting position.
     *
     * @param from the current position of the indexer
     * @return the time in milliseconds that the indexer should wait for the servo to move from
     *         the current position to the target shooting position, based on the distance between
     *         the positions
     */
    private double calculateShootingWaitTime(Position from) {
        // If already at a shooting position, calculate time to next shooting position
        if (from == Position.SHOOT_1 || from == Position.SHOOT_2 || from == Position.SHOOT_3) {
            return SHOOT_DELAY_TO_NEXT_POS;
        }

        // Moving from intake position to shooting position
        return SHOOT_DELAY_TO_CENTER;
    }

    /**
     * Sets the position of the indexer servo to the specified position.
     *
     * @param position the position to set for the indexer
     */
    private void setPosition(@NonNull Position position) {
        indexer.setPosition(position.value());
    }

    /**
     * Frees any resources used by the indexer, such as the color processor.
     */
    @Override
    public void close() {
        colorProcessor.close();
    }

    /**
     * Enum representing the different positions of the indexer servo, with
     * corresponding position values for both intake and shooting.
     */
    private enum Position {
        INTAKE_1(Indexer.POSITION_1_INTAKE), INTAKE_2(Indexer.POSITION_2_INTAKE), INTAKE_3(Indexer.POSITION_3_INTAKE),
        SHOOT_1(Indexer.POSITION_1_SHOOT), SHOOT_2(Indexer.POSITION_2_SHOOT), SHOOT_3(Indexer.POSITION_3_SHOOT);

        private final double positionValue;

        /**
         * Constructor for the Position enum, which assigns a specific position value to
         * each enum constant.
         *
         * @param positionValue the position value corresponding to the enum constant,
         *                      used to set the servo position for intake or shooting
         */
        Position(double positionValue) {
            this.positionValue = positionValue;
        }

        /**
         * Returns the position value associated with the enum constant, which can be
         * used to set the servo position for intake or shooting.
         *
         * @return the position value associated with the enum constant
         */
        public double value() {
            return positionValue;
        }
    }

    /**
     * Enum representing the different states of the indexer mechanism.
     */
    public enum State {
        INACTIVE,
        ROTATING,
        INTAKING,
        SHOOTING
    }
}
