package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.SharedState;

import java.util.List;

/**
 * Controls the turret motor and manages Limelight-based tracking.
 *
 * HOW TURRET TRACKING WORKS (step by step):
 *
 * 1. CALCULATE ANGLE TO TARGET:
 *    We know where the basket is on the field (targetX, targetY).
 *    We know where the robot is (from odometry: robotX, robotY).
 *    Using trigonometry (atan), we calculate the angle from the robot to the basket.
 *
 * 2. ACCOUNT FOR ROBOT HEADING:
 *    The turret is mounted ON the robot, so if the robot is rotated,
 *    the turret needs to compensate. We subtract the robot's heading
 *    so the turret angle is relative to the robot body, not the field.
 *
 * 3. CONVERT TO ENCODER TICKS:
 *    The turret motor uses encoder ticks for position control.
 *    We multiply the angle in degrees by the ticks-per-degree constant.
 *
 * 4. SAFETY CLAMP:
 *    We check that the target position is within the turret's physical
 *    rotation limits (EXTREME_LEFT and EXTREME_RIGHT) to prevent damage.
 *
 * 5. LIMELIGHT FINE-TUNING:
 *    Once the Limelight detects an AprilTag, we use its tx (horizontal error)
 *    value to make small corrections for more accurate aiming.
 *
 * TODO: [Student] Draw a diagram showing the angle calculation.
 *       Label the robot position, basket position, and the angle between them.
 *
 * TODO: [Student] What is the difference between pipeline 0 and pipeline 1
 *       on the Limelight? When do we switch between them?
 */
public class TurretSubsystem {

    private final DcMotor turretMotor;
    private final Limelight3A limelight;
    private final double targetX;
    private final double targetY;

    /** Encoder ticks per degree of turret rotation. */
    private static final double M = RobotHardware.TURRET_TICKS_PER_DEGREE;

    /** Maximum encoder position (leftmost rotation). */
    private static final int EXTREME_LEFT = RobotHardware.TURRET_EXTREME_LEFT;

    /** Minimum encoder position (rightmost rotation). */
    private static final int EXTREME_RIGHT = RobotHardware.TURRET_EXTREME_RIGHT;

    // State
    private boolean motifDetected = false;
    private String motif = "";
    private int turretPose = 0;

    /**
     * Creates a TurretSubsystem.
     *
     * @param turretMotor  the turret DcMotor from RobotHardware
     * @param limelight    the Limelight3A camera from RobotHardware
     * @param targetX      X coordinate of the target basket (from AllianceConfig)
     * @param targetY      Y coordinate of the target basket (from AllianceConfig)
     */
    public TurretSubsystem(DcMotor turretMotor, Limelight3A limelight,
                           double targetX, double targetY) {
        this.turretMotor = turretMotor;
        this.limelight = limelight;
        this.targetX = targetX;
        this.targetY = targetY;
    }

    /**
     * Updates turret position based on the robot's current pose and Limelight data.
     * Must be called every loop iteration for smooth tracking.
     *
     * @param robotX       current robot X position (inches, from follower)
     * @param robotY       current robot Y position (inches, from follower)
     * @param robotHeading current robot heading (radians, from follower)
     * @param telemetry    for debug output to Driver Station
     * @param gamepad1     for rumble feedback when locked on target
     * @param gamepad2     for rumble feedback when locked on target
     */
    public void update(double robotX, double robotY, double robotHeading,
                       Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {

        // -- Step 1: Calculate angle from robot to target --
        double targetAngleDeg;
        if (motifDetected) {
            // After detection: aim at the actual basket position
            targetAngleDeg = calculateAngleToTarget(robotX, robotY, targetX, targetY);
        } else {
            // Before detection: aim at field center (72, 144) as a fallback
            // This points the Limelight in the general direction of the AprilTags
            targetAngleDeg = calculateAngleToTarget(robotX, robotY, 72, 144);
        }

        // -- Step 2: Convert field angle to turret angle --
        // Subtract robot heading so turret angle is relative to robot body
        double robotHeadingDeg = Math.toDegrees(robotHeading);
        double turretAngleDeg = targetAngleDeg - (robotHeadingDeg - 90);

        // -- Step 3: Convert degrees to encoder ticks --
        turretPose = (int) (turretAngleDeg * M);

        // -- Step 4: Safety clamp --
        if (turretPose > EXTREME_LEFT || turretPose < EXTREME_RIGHT) {
            // Target is outside physical range, don't move
            return;
        }

        // -- Step 5: Process Limelight data --
        LLResult result = limelight.getLatestResult();
        double deadband = 2.0; // degrees of error we consider "close enough"

        if (result != null && result.isValid()) {
            telemetry.addData("Turret Error", result.getTx());

            double error = result.getTx();

            // Rumble controllers when we're locked on target
            if (Math.abs(error) < deadband) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }

            // Detect motif from AprilTag if we haven't yet
            if (!motifDetected) {
                detectMotifFromAprilTag(result);
            }
        }

        // -- Move turret to calculated position --
        turretMotor.setTargetPosition(turretPose);
    }

    /**
     * Calculates the angle from the robot to a target point on the field.
     *
     * Uses the atan (arctangent) function to find the angle.
     * The result is normalized to [0, 180) degrees using the modular formula.
     *
     * TODO: [Student] Why do we use ((result % 180) + 180) % 180 instead
     *       of just Math.toDegrees(Math.atan(...))? What problem does the
     *       modular arithmetic solve? Hint: atan can return negative values.
     *
     * @param robotX   robot X position
     * @param robotY   robot Y position
     * @param targetX  target X position
     * @param targetY  target Y position
     * @return angle in degrees [0, 180)
     */
    private double calculateAngleToTarget(double robotX, double robotY,
                                           double targetX, double targetY) {
        double rawAngle = Math.toDegrees(
            Math.atan((targetY - robotY) / (targetX - robotX))
        );
        return ((rawAngle % 180) + 180) % 180;
    }

    /**
     * Attempts to detect the game motif from Limelight AprilTag fiducials.
     *
     * AprilTag IDs and their meanings:
     *   Tag 21 = Motif "GPP" (Green in position 1)
     *   Tag 22 = Motif "PGP" (Green in position 2)
     *   Tag 23 = Motif "PPG" (Green in position 3)
     *
     * Once a motif is detected, the Limelight switches from pipeline 0
     * (AprilTag detection) to pipeline 1 (tracking mode).
     *
     * TODO: [Student] What does the motif tell us about how to score?
     *       Why do we only need to detect it once?
     *
     * @param result  the Limelight result containing fiducial data
     */
    private void detectMotifFromAprilTag(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (!fiducials.isEmpty()) {
            int tagId = fiducials.get(0).getFiducialId();
            if (tagId == 21) {
                motif = "GPP";
            } else if (tagId == 22) {
                motif = "PGP";
            } else if (tagId == 23) {
                motif = "PPG";
            }
            SharedState.motif = motif;
            limelight.pipelineSwitch(1);
            motifDetected = true;
        }
    }

    // -----------------------------------------------------------------------
    // Getters
    // -----------------------------------------------------------------------

    /** Returns the detected motif string (e.g., "GPP"), or "" if not yet detected. */
    public String getMotif() {
        return motif;
    }

    /** Returns true if the motif has been detected from an AprilTag. */
    public boolean isMotifDetected() {
        return motifDetected;
    }

    /** Returns the current turret encoder target position. */
    public int getTurretPose() {
        return turretPose;
    }

    /**
     * Sets the motif directly (e.g., when loading from SharedState in TeleOp).
     *
     * @param motif  the motif string (e.g., "GPP")
     */
    public void setMotif(String motif) {
        this.motif = motif;
        if (!motif.isEmpty()) {
            this.motifDetected = true;
        }
    }
}
