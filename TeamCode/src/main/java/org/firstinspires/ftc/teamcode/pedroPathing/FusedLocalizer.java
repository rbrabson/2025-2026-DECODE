package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.MathEx;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/**
 * FusedLocalizer is a localization system that combines odometry data from a pinpoint localizer
 * with vision data from a Limelight camera. It performs sensor fusion to provide a more accurate
 * estimate of the robot's pose on the field, especially in situations where odometry may drift or
 * when the robot can see multiple fiducial tags.
 * <p>
 * The localizer operates in two modes: AUTO and TELEOP, which affect the velocity gating thresholds
 * for accepting vision data. The fusion process blends the X and Y coordinates from odometry and
 * vision based on the number of detected tags, while keeping the heading from odometry to ensure
 * stability.
 */
public class FusedLocalizer implements Localizer {
    // =================== Tunable Constants ===================
    private static final double FIELD_LIMIT_IN = 144.0;      // Field boundary in inches
    private static final double MAX_JUMP_IN = 24.0;          // Max allowed vision jump in inches

    // Angular velocity gating
    private static final double MAX_AUTO_ANGULAR_VEL = 2.0;   // rad/sec
    private static final double MAX_TELEOP_ANGULAR_VEL = 4.0; // rad/sec

    private static final double BASE_BLEND = 0.35;            // Base vision/odometry blend factor
    private static final double MIN_VALID_BLEND = 0.05;       // Minimum blend to accept vision
    private static final long VISION_COOLDOWN_MS = 50;        // Vision update cooldown

    // Latency compensation
    private static final long LIMELIGHT_LATENCY_MS = 40;      // Limelight latency in ms
    private static final long HISTORY_MS = 200;               // Pose history window in ms


    private final Localizer odometry;
    private final Limelight3A limelight;
    private final Deque<TimedPose> poseHistory = new ArrayDeque<>();
    private Pose fusedPose = new Pose(0, 0, 0);
    private volatile Mode mode = Mode.TELEOP;

    private long lastVisionUpdateTime = 0;

    /**
     * Constructor for FusedLocalizer.
     *
     * @param odometry  The underlying localizer providing odometry data (e.g., pinpoint localizer)
     * @param limelight The Limelight3A instance providing vision data
     */
    public FusedLocalizer(@NonNull Localizer odometry, @NonNull Limelight3A limelight) {
        this.odometry = odometry;
        this.limelight = limelight;
    }

    /**
     * Sets the operating mode of the localizer, which affects vision data acceptance thresholds.
     *
     * @param mode The desired mode (AUTO or TELEOP)
     * @return The FusedLocalizer instance for chaining
     */
    public FusedLocalizer withMode(Mode mode) {
        this.mode = mode;
        return this;
    }

    // ================= REQUIRED INTERFACE =================
    @Override
    public Pose getPose() {
        return fusedPose;
    }

    @Override
    public void setPose(Pose setPose) {
        odometry.setPose(setPose);
        fusedPose = setPose;
    }

    /**
     * Updates the localizer by fusing odometry and vision data. This method should be called
     * regularly in the main loop. It performs several checks on the vision data, computes a
     * blend factor based on the number and quality of detected tags, and then fuses the X/Y
     * coordinates while keeping the heading from odometry. It also includes latency compensation
     * by using a history of past poses to estimate where the robot was when the vision data was
     * captured.
     */
    @Override
    public void update() {
        odometry.update();
        long now = System.currentTimeMillis();

        // Store pose history for latency compensation
        addPoseHistory(odometry.getPose(), now);

        Pose currentPose = odometry.getPose();
        LLResult result = limelight.getLatestResult();

        if (!visionBasicValid(result, currentPose)) return;
        if (now - lastVisionUpdateTime < VISION_COOLDOWN_MS) return;
        lastVisionUpdateTime = now;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        int tagCount = tags != null ? tags.size() : 0;
        Pose visionPose = getRobotPoseFromCamera(result.getBotpose());

        double angularVel = Math.abs(odometry.getVelocity().getHeading());
        double maxAngularVel = mode == Mode.AUTO ? MAX_AUTO_ANGULAR_VEL : MAX_TELEOP_ANGULAR_VEL;
        double rotationFactor = 1.0 - Math.min(1.0, angularVel / maxAngularVel);
        rotationFactor = Math.max(0.1, rotationFactor);

        double blend = computeBlend(tags) * rotationFactor;
        if (blend < MIN_VALID_BLEND) return;

        // Latency compensation
        long captureTime = now - LIMELIGHT_LATENCY_MS;
        Pose pastPose = getPoseAtTime(captureTime);

        double correctionFactor = computeCorrectionConfidence(pastPose, visionPose);
        blend *= correctionFactor;

        // Speed scaling
        double speed = Math.hypot(odometry.getVelocity().getX(), odometry.getVelocity().getY());
        blend *= (speed < 2.0) ? 1.5 : Math.min(1.0, 2.0 / (speed + 0.1));
        blend = Math.min(1.0, blend);

        // Fuse X/Y at past pose
        Pose fusedPast = fuseXY(pastPose, visionPose, blend);

        // Heading fusion
        double spreadBonus = computeTagSpreadBonus(tags);
        double blendedHeading = computeBlendedHeading(tagCount, angularVel, pastPose, visionPose, blend, spreadBonus);

        fusedPast = new Pose(fusedPast.getX(), fusedPast.getY(), blendedHeading);
        fusedPose = forwardCorrectPose(currentPose, fusedPast, pastPose);

        odometry.setPose(fusedPose);
    }

    /**
     * Adds a new pose to the history for latency compensation and removes old poses that are outside
     * the defined history window. This allows the localizer to estimate where the robot was at the
     * time when the vision data was captured.
     *
     * @param pose The new pose to add to the history
     * @param time The timestamp associated with the new pose, used for determining when to remove old poses
     */
    private void addPoseHistory(Pose pose, long time) {
        poseHistory.addLast(new TimedPose(pose, time));
        while (!poseHistory.isEmpty()) {
            TimedPose first = poseHistory.peekFirst();
            if (first == null || time - first.time <= HISTORY_MS) {
                break;
            }
            poseHistory.removeFirst();
        }
    }

    /**
     * Performs basic validity checks on the vision result, including field bounds, jump filtering,
     * speed-based single tag rejection, and tag area spread checks. This helps to filter out
     * obviously bad vision data before attempting to fuse it with odometry.
     *
     * @param result  The latest vision result from the Limelight
     * @param odoPose The current pose from odometry, used for jump filtering
     * @return True if the vision data passes basic validity checks, false otherwise
     */
    private boolean visionBasicValid(LLResult result, Pose odoPose) {
        if (result == null || !result.isValid()) {
            return false;
        }

        Pose3D llPose = result.getBotpose();
        if (llPose == null) {
            return false;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return false;
        }

        // Field bounds
        double llX = llPose.getPosition().x * 39.3701;
        double llY = llPose.getPosition().y * 39.3701;
        if (Math.abs(llX) > FIELD_LIMIT_IN || Math.abs(llY) > FIELD_LIMIT_IN) {
            return false;
        }

        // Apply jump filter
        double jump = Math.hypot(llX - odoPose.getX(), llY - odoPose.getY());
        if (jump > MAX_JUMP_IN) {
            return false;
        }

        // Speed-based single tag rejection
        double speed = Math.hypot(odometry.getVelocity().getX(), odometry.getVelocity().getY());
        if (tags.size() == 1 && speed > 10) {
            return false;
        }

        // Tag area spread check
        if (tags.size() >= 2) {
            double minArea = Double.MAX_VALUE;
            double maxArea = 0.0;
            for (LLResultTypes.FiducialResult tag : tags) {
                double area = tag.getTargetArea();
                minArea = Math.min(minArea, area);
                maxArea = Math.max(maxArea, area);
            }
            return !(maxArea - minArea < 0.01);
        }

        return true;
    }

    /**
     * Converts the robot pose from the camera's coordinate system to the field coordinate system
     * used by the localizer. This involves extracting the X/Y position and yaw from the camera's
     * pose, converting units from meters to inches, and applying the appropriate coordinate
     * transformations to align with the localizer's coordinate system. The resulting Pose will
     * have the same heading as the camera's yaw but will be positioned according to the camera's
     * observed position of the robot on the field.
     *
     * @param botpose The Pose3D representing the robot's pose in the camera's coordinate system,
     *                obtained from the Limelight result
     * @return A Pose representing the robot's pose in the field coordinate system, with X/Y converted to
     *         inches and heading derived from the camera's yaw, ready to be fused with odometry data
     */
    @NonNull
    private Pose getRobotPoseFromCamera(@NonNull Pose3D botpose) {
        return new Pose(
                botpose.getPosition().toUnit(DistanceUnit.INCH).x,
                botpose.getPosition().toUnit(DistanceUnit.INCH).y,
                botpose.getOrientation().getYaw(AngleUnit.RADIANS),
                FTCCoordinates.INSTANCE
        ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    /**
     * Computes a blend factor for fusing vision data based on the number and quality of detected tags.
     * It considers the total area of detected tags as a proxy for confidence, applies nonlinear scaling
     * to increase the influence of stronger detections, and includes a boost for close-range detections.
     *
     * @param tags The list of detected fiducial tags from the vision result
     * @return A blend factor between 0.0 and 1.0 that determines how much to trust the vision data
     *         when fusing with odometry
     */
    private double computeBlend(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) {
            return 0.0;
        }

        double totalArea = 0.0;
        for (LLResultTypes.FiducialResult tag : tags) {
            totalArea += tag.getTargetArea();
        }

        double areaConfidence = Math.min(1.0, totalArea / 0.15);
        double countConfidence = Math.min(1.0, tags.size() / 2.0);

        double confidence = 0.6 * areaConfidence + 0.4 * countConfidence;
        confidence = confidence * confidence;
        if (areaConfidence > 0.8) {
            confidence *= 1.25;
        }

        return BASE_BLEND * confidence;
    }

    /**
     * Retrieves the pose from the history that corresponds to the given target time, performing linear
     * interpolation between the two closest poses if necessary.
     *
     * @param targetTime The timestamp for which to retrieve the pose, typically the time when the
     *                   vision data was captured
     * @return The Pose from the history that corresponds to the target time, obtained by interpolating between
     *         the two closest poses if the target time falls between them, or the closest pose if the
     *         target time is outside the range of the history
     */
    private Pose getPoseAtTime(long targetTime) {
        TimedPose prev = null;
        for (TimedPose tp : poseHistory) {
            if (tp.time > targetTime) {
                if (prev == null) {
                    return tp.pose;
                }

                double t = (targetTime - prev.time) / (double) (tp.time - prev.time);
                double x = MathEx.lerp(prev.pose.getX(), tp.pose.getX(), t);
                double y = MathEx.lerp(prev.pose.getY(), tp.pose.getY(), t);
                double heading = prev.pose.getHeading() + MathEx.angleWrap(tp.pose.getHeading() - prev.pose.getHeading()) * t;
                return new Pose(x, y, heading);
            }
            prev = tp;
        }
        return prev != null ? prev.pose : odometry.getPose();
    }

    /**
     * Computes a confidence factor for the vision correction based on how far the vision pose is from
     * the odometry pose at the time of capture. This helps to reduce the influence of vision data
     * that suggests a large jump from the expected position, which is more likely to be an error
     * than a true correction.
     *
     * @param odo    The odometry pose at the time of vision capture (after latency compensation)
     * @param vision The pose estimated from vision data
     * @return A confidence factor between 0.1 and 1.0 that reduces the blend of vision data if it
     *         suggests a large correction
     */
    private double computeCorrectionConfidence(@NonNull Pose odo, @NonNull Pose vision) {
        double dx = vision.getX() - odo.getX();
        double dy = vision.getY() - odo.getY();
        double magnitude = Math.hypot(dx, dy);

        double maxCorrection = 24.0;
        double confidence = 1.0 - (magnitude / maxCorrection);

        return Range.clip(confidence, 0.1, 1.0);
    }

    /**
     * Fuses the X and Y coordinates from odometry and vision based on the provided blend factor,
     * while keeping the heading from odometry. This allows the localizer to correct for positional
     * drift while maintaining stable heading estimates, which are often more reliable from odometry.
     *
     * @param odo    The odometry pose (used for heading and as a base for blending)
     * @param vision The vision pose (used for X/Y coordinates)
     * @param blend  The blend factor between 0.0 and 1.0 that determines how much to trust the vision data
     * @return A new Pose that combines the X/Y from vision and the heading from odometry, blended
     *         according to the blend factor
     */
    @NonNull
    private Pose fuseXY(@NonNull Pose odo, @NonNull Pose vision, double blend) {
        double x = MathEx.lerp(odo.getX(), vision.getX(), blend);
        double y = MathEx.lerp(odo.getY(), vision.getY(), blend);
        return new Pose(x, y, odo.getHeading());
    }

    /**
     * Computes a bonus factor for the blend based on how spread out the detected tags are in the
     * camera's view. A wider spread of tags generally indicates a more reliable pose estimate,
     * while tags that are clustered together may be more susceptible to noise and errors.
     *
     * @param tags The list of detected fiducial tags from the vision result, used to calculate the
     *             spread bonus
     * @return A bonus factor greater than or equal to 1.0 that increases the blend of vision data
     *         when the tags are well spread out, and is equal to 1.0 when there are fewer than 2
     *         tags or when the tags are very close together
     */
    private double computeTagSpreadBonus(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.size() < 2) {
            return 1.0;
        }

        double maxDistance = 0.0;
        double[] xs = new double[tags.size()];
        double[] ys = new double[tags.size()];

        for (int i = 0; i < tags.size(); i++) {
            LLResultTypes.FiducialResult tag = tags.get(i);
            Pose3D robotSpace = tag.getTargetPoseRobotSpace();
            if (robotSpace == null) {
                xs[i] = 0.0;
                ys[i] = 0.0;
            } else {
                xs[i] = robotSpace.getPosition().toUnit(DistanceUnit.INCH).x;
                ys[i] = robotSpace.getPosition().toUnit(DistanceUnit.INCH).y;
            }
        }

        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                double dx = xs[i] - xs[j];
                double dy = ys[i] - ys[j];
                double dist = Math.hypot(dx, dy);
                if (dist > maxDistance) {
                    maxDistance = dist;
                }
            }
        }

        double minSpread = 2.0;   // inches
        double maxSpread = 24.0;  // inches
        return 1.0 + 0.5 * Range.clip((maxDistance - minSpread) / (maxSpread - minSpread), 0.0, 1.0);
    }

    /**
     * Computes a blended heading based on the number of detected tags, the robot's angular velocity,
     * and the spread of the tags. If there are multiple tags and the robot is relatively stationary,
     * it trusts the vision heading directly. Otherwise, it blends the vision and odometry headings
     * based on the provided blend factor and a bonus from the tag spread.
     *
     * @param tagCount    The number of detected tags, which influences how much to trust the
     *                    vision heading
     * @param angularVel  The current angular velocity of the robot, used to gate trusting vision
     *                    heading when the robot is rotating
     * @param pastPose    The odometry pose at the time of vision capture, used as a base for
     *                    blending
     * @param visionPose  The pose estimated from vision data, used for blending the heading
     * @param blend       The blend factor between 0.0 and 1.0 that determines how much to trust
     *                    the vision data for X/Y, which also influences heading blending
     * @param spreadBonus A bonus factor based on how spread out the detected tags are, which can
     *                    increase the blend of vision data and thus the influence of the vision
     *                    heading
     * @return A blended heading that combines the odometry and vision headings according to the
     *         number of tags, angular velocity, blend factor, and tag spread bonus, with a preference
     *         for trusting vision heading directly when there are multiple tags and the robot is not
     *         rotating significantly
     */
    private double computeBlendedHeading(int tagCount, double angularVel, Pose pastPose, Pose visionPose, double blend, double spreadBonus) {
        if (tagCount >= 2 && angularVel < 1.0) {
            return visionPose.getHeading();
        } else {
            double headingBlend = Math.min(1.0, blend * 1.5 * spreadBonus);
            return blendHeading(pastPose.getHeading(), visionPose.getHeading(), headingBlend);
        }
    }

    /**
     * Performs forward correction of the current pose based on the fused past pose and the past
     * odometry pose.
     *
     * @param currentPose The current pose from odometry, which will be corrected based on the
     *                    difference between the fused past pose and the past odometry pose
     * @param fusedPast   The fused pose at the time of vision capture, which combines odometry
     *                    and vision data
     * @param pastPose    The odometry pose at the time of vision capture, used as a reference for
     *                    how much the robot has moved since then
     * @return A new Pose that applies the correction from the fused past pose to the current pose,
     *         effectively "forward correcting" the current pose based on the vision correction applied
     *         at the past pose
     */
    @NonNull
    private Pose forwardCorrectPose(@NonNull Pose currentPose, @NonNull Pose fusedPast, @NonNull Pose pastPose) {
        double dx = fusedPast.getX() - pastPose.getX();
        double dy = fusedPast.getY() - pastPose.getY();
        double dHeading = MathEx.angleWrap(fusedPast.getHeading() - pastPose.getHeading());

        double correctedX = currentPose.getX() + dx;
        double correctedY = currentPose.getY() + dy;
        double correctedHeading = currentPose.getHeading() + dHeading;

        return new Pose(correctedX, correctedY, correctedHeading);
    }

    /**
     * Blends the heading from odometry and vision based on the provided blend factor. This method
     * performs angle wrapping to ensure that the blended heading is smooth and does not suffer from
     * discontinuities when crossing the -pi to pi boundary. The blend factor determines how much to
     * trust the vision heading versus the odometry heading, with a higher blend giving more weight
     * to the vision heading.
     *
     * @param odo    The odometry heading (used as a base for blending)
     * @param vision The vision heading (used for blending)
     * @param blend  The blend factor between 0.0 and 1.0 that determines how much to trust the
     *               vision heading
     * @return A blended heading that combines the odometry and vision headings according to the
     *         blend factor, with proper angle wrapping to ensure smooth transitions
     */
    private double blendHeading(double odo, double vision, double blend) {
        double diff = MathEx.angleWrap(vision - odo);
        return odo + diff * blend;
    }

    @Override
    public double getTotalHeading() {
        return odometry.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return odometry.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odometry.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odometry.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        odometry.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return odometry.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return odometry.isNAN();
    }

    @Override
    public Pose getVelocity() {
        return odometry.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return odometry.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odometry.setStartPose(setStart);
        fusedPose = setStart;
    }

    public enum Mode {AUTO, TELEOP}

    // ================= Pose History for Latency Compensation =================
    private static class TimedPose {
        Pose pose;
        long time;

        TimedPose(Pose pose, long time) {
            this.pose = pose;
            this.time = time;
        }
    }
}
