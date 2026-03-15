package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.utils.MathEx;

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
    // ====== Tunable Constants ======
    private static final double FIELD_LIMIT_IN = 144.0;
    private static final double MAX_JUMP_IN = 24.0;

    // Angular gating (better for mecanum)
    private static final double MAX_AUTO_ANGULAR_VEL = 2.0;   // rad/sec
    private static final double MAX_TELEOP_ANGULAR_VEL = 4.0; // rad/sec

    private static final double BASE_BLEND = 0.35;

    private final Localizer odometry;
    private final Limelight3A limelight;

    private Pose fusedPose = new Pose(0, 0, 0);

    public enum Mode { AUTO, TELEOP }

    private volatile Mode mode = Mode.TELEOP;

    /**
     * Creates a new FusedLocalizer that combines odometry from the PedroPath localizer with
     * vision data from the Limelight.
     *
     * @param odometry  the localizer that provides odometry data
     * @param limelight the Limelight camera providing vision data
     */
    public FusedLocalizer(@NonNull Localizer odometry, @NonNull Limelight3A limelight) {
        this.odometry = odometry;
        this.limelight = limelight;
    }

    /**
     * Sets the operating mode of the localizer, which affects velocity gating thresholds.
     *
     * @param mode the mode to set (AUTO or TELEOP). Defaults to TELEOP.
     * @return this FusedLocalizer instance for chaining
     */
    public FusedLocalizer withMode(Mode mode) {
        this.mode = mode;
        return this;
    }

    // ================= UPDATE METHOD ================

    @Override
    public void update() {
        // Update odometry
        odometry.update();

        Pose odoPose = odometry.getPose();
        fusedPose = odoPose; // fallback if vision invalid

        // Get vision result
        LLResult result = limelight.getLatestResult();
        if (!visionBasicValid(result, odoPose)) {
            return;
        }

        // Angular velocity gating (reduce vision influence on fast rotation)
        double angularVel = Math.abs(odometry.getVelocity().getHeading());
        double maxAngularVel = mode == Mode.AUTO ? MAX_AUTO_ANGULAR_VEL : MAX_TELEOP_ANGULAR_VEL;
        double rotationFactor = 1.0 - Math.min(1.0, angularVel / maxAngularVel);
        rotationFactor = Math.max(0.1, rotationFactor); // minimal influence even if rotating fast

        // Base blend from tag count
        double blend = computeBlend(result) * rotationFactor;

        // Scale by correction confidence (how far vision is from odometry)
        Pose visionPose = getRobotPoseFromCamera(result.getBotpose());
        double correctionFactor = computeCorrectionConfidence(odoPose, visionPose);
        blend *= correctionFactor;

        // Stationary boost for precise corrections when slow, or dynamic scaling when fast —
        // these are mutually exclusive to avoid stacking multipliers
        double speed = Math.hypot(odometry.getVelocity().getX(), odometry.getVelocity().getY());
        if (speed < 2.0) {
            blend *= 1.5; // stationary boost only
        } else {
            blend *= Math.min(1.0, 2.0 / (speed + 0.1)); // dynamic scaling only at speed
        }

        // Clamp blend to 0–1
        blend = Math.min(1.0, blend);

        // Fuse X/Y and heading
        Pose fusedXY = fuseXY(odoPose, visionPose, blend);
        double blendedHeading = blendHeading(odoPose.getHeading(), visionPose.getHeading(), blend);
        fusedPose = new Pose(fusedXY.getX(), fusedXY.getY(), blendedHeading);

        // Update PedroPath with fused pose
        odometry.setPose(fusedPose);
    }

    // ================= POSE METHODS =================

    @Override
    public Pose getPose() {
        return fusedPose;
    }

    @Override
    public void setPose(Pose setPose) {
        odometry.setPose(setPose);
        fusedPose = setPose;
    }

    @Override
    public void setStartPose(Pose setStart) {
        odometry.setStartPose(setStart);
        fusedPose = setStart;
    }

    // ================= DELEGATED METHODS =================

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

    // ================= VISION PROCESSING =================

    /**
     * Performs basic validation on the vision result to ensure it is reasonable before attempting
     * to fuse it with odometry. This includes checks for validity, field limits, and jump distance
     * from odometry.
     *
     * @param result  the latest vision result from the Limelight
     * @param odoPose the current pose from odometry to compare against
     * @return true if the vision result passes basic validation checks, false otherwise
     */
    private boolean visionBasicValid(LLResult result, Pose odoPose) {
        if (result == null || !result.isValid()) return false;

        Pose3D llPose = result.getBotpose();
        if (llPose == null) return false;

        // Get X and Y, converting meters to inches
        double llX = llPose.getPosition().x * 39.3701;
        double llY = llPose.getPosition().y * 39.3701;

        if (Math.abs(llX) > FIELD_LIMIT_IN || Math.abs(llY) > FIELD_LIMIT_IN) {
            return false;
        }

        double jump = Math.hypot(llX - odoPose.getX(), llY - odoPose.getY());

        return jump <= MAX_JUMP_IN;
    }

    /**
     * Computes a confidence score for the vision correction based on how far the vision pose is from
     * the odometry pose. The confidence decreases as the correction magnitude increases, with a
     * tunable maximum reasonable correction distance. This confidence score can be used to scale the
     * blend factor for fusing vision and odometry, ensuring that large, potentially erroneous
     * corrections from vision have less influence on the final pose estimate.
     *
     * @param odo the current pose from odometry to compare against
     * @param vision the pose extracted from vision to compare against odometry
     * @return a confidence score between 0.1 and 1.0 that indicates how much to trust the vision
     *         correction, with higher values indicating more confidence in the vision data
     */
    private double computeCorrectionConfidence(@NonNull Pose odo, @NonNull Pose vision) {
        double dx = vision.getX() - odo.getX();
        double dy = vision.getY() - odo.getY();
        double correctionMagnitude = Math.hypot(dx, dy);

        // If vision disagrees by 2 feet, confidence should be near the minimum
        double maxReasonableCorrection = 24.0;
        double confidence = 1.0 - (correctionMagnitude / maxReasonableCorrection);

        return Range.clip(confidence, 0.1, 1.0);
    }

    /**
     * Computes the blend factor for fusing vision and odometry based on the number of detected
     * fiducial tags. The more tags detected, the higher the confidence in the vision data, and
     * thus the higher the blend factor. The blend factor is scaled by a base blend constant to
     * ensure that vision does not completely override odometry, providing a balance between the
     * two sources of information.
     *
     * @param result the latest vision result from the Limelight, used to determine the number of
     *               detected tags
     * @return a blend factor between 0 and BASE_BLEND that determines how much to trust vision
     *         vs odometry
     */
    private double computeBlend(@NonNull LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        int count = tags != null ? tags.size() : 0;
        double confidence = Math.min(1.0, count / 2.0);

        return BASE_BLEND * confidence;
    }

    /**
     * Fuses the X and Y coordinates from odometry and vision based on the provided blend factor,
     * while keeping the heading from odometry to ensure stability.
     *
     * @param odo    the pose from odometry
     * @param vision the pose from vision
     * @param blend  the blend factor between 0 and 1 that determines how much to trust vision vs
     *               odometry
     * @return a new Pose that is a fusion of the odometry and vision poses, with X and Y blended
     *         and heading from odometry
     */
    @NonNull
    private Pose fuseXY(@NonNull Pose odo, @NonNull Pose vision, double blend) {
        double x = MathEx.lerp(odo.getX(), vision.getX(), blend);
        double y = MathEx.lerp(odo.getY(), vision.getY(), blend);

        return new Pose(x, y, odo.getHeading());
    }

    /**
     * Blends the heading from odometry and vision based on the provided blend factor. The method
     * calculates the angular difference between the vision and odometry headings, wraps it to the
     * range [-pi, pi], and then applies the blend factor to determine how much of the vision heading
     * to incorporate into the final heading.
     *
     * @param odo    the heading from odometry
     * @param vision the heading from vision
     * @param blend  the blend factor between 0 and 1 that determines how much to trust vision vs
     *               odometry for heading
     * @return a blended heading that incorporates both odometry and vision, with the influence of
     *         vision determined by the blend factor
     */
    private double blendHeading(double odo, double vision, double blend) {
        double diff = MathEx.angleWrap(vision - odo);
        return odo + diff * blend;
    }

    /**
     * Converts the X and Y coordinates from the Limelight's coordinate system (where X is
     * forward and Y is left) to the standard PedroPathing coordinate system (where X is right
     * and Y is forward). The heading is also converted from the Limelight's orientation to the
     * PedroPathing coordinate system.
     *
     * @param botpose the Pose3D from the Limelight, containing position in meters and orientation
     *               as a quaternion
     * @return a Pose representing the position and heading in the PedroPathing coordinate system,
     *         with X as right, Y as forward, and heading adjusted accordingly
     */
    @NonNull
    private Pose getRobotPoseFromCamera(@NonNull Pose3D botpose) {
        return getRobotPoseFromCamera(
                botpose.getPosition().toUnit(DistanceUnit.INCH).x,
                botpose.getPosition().toUnit(DistanceUnit.INCH).y,
                botpose.getOrientation().getYaw(AngleUnit.RADIANS));
    }

    /**
     * Converts the X and Y coordinates from the Limelight's coordinate system (where X is
     * forward and Y is left) to the standard PedroPathing coordinate system (where X is right
     * and Y is forward). The heading is also converted from the Limelight's orientation to the
     * PedroPathing coordinate system.
     *
     * @param x          the X coordinate from the Limelight, representing forward distance in inches
     * @param y          the Y coordinate from the Limelight, representing left distance in inches
     * @param turnInRad  the heading from the Limelight in radians, where 0 is facing forward and
     *                   positive is left
     * @return a Pose representing the position and heading in the PedroPathing coordinate system,
     *         with X as right, Y as forward, and heading adjusted accordingly
     */
    @NonNull
    private Pose getRobotPoseFromCamera(double x, double y, double turnInRad) {
        return new Pose(x, y, turnInRad, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
