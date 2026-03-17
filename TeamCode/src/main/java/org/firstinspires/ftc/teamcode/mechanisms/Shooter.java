package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Hood;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.robotcontrol.ShooterController;

import java.util.Objects;

/**
 * Shooter mechanism class controlling turret, flywheel, and hood.
 * Fully LUT-driven via ShooterController.
 * <p>
 * <b>Note</b>: You must call setLocalizer() and setAlliancePose() before using the shooter.
 */
public class Shooter implements Mechanism {
    private static final Pose BLUE_GOAL_SCORE = new Pose(25, 18.5, 0);
    private static final Pose RED_GOAL_SCORE = new Pose(25, 125.5, 0);

    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Telemetry telemetry;
    private ShooterController shooterModel;
    private Localizer localizer;
    private Alliance alliance;

    /**
     * Initializes shooter subsystems and telemetry.
     *
     * @param hardwareMap HardwareMap for accessing hardware devices
     * @param telemetry   Telemetry for logging and debugging
     */
    public Shooter(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap);
        this.telemetry = Objects.requireNonNull(telemetry);
        this.turret = new Turret(map, telemetry);
        this.flywheel = new Flywheel(map, telemetry);
        this.hood = new Hood(map, telemetry);
        shooterModel = new ShooterController();
        telemetry.addLine("Shooter initialized");
    }

    /**
     * Fluent interface method to set the localizer for the shooter mechanism. This allows for chaining
     * this configuration after constructing the Shooter object, providing flexibility in how the shooter
     * is initialized and configured.
     *
     * @param localizer The Localizer instance that provides the current pose and velocity of the
     *                  robot, which is used by the shooter model to calculate the necessary adjustments
     *                  for aiming and shooting.
     * @return The Shooter instance with the updated localizer, allowing for method chaining.
     */
    public Shooter setLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
        return this;
    }

    /**
     * Fluent interface method to set alliance and starting pose for the shooter model. This
     * allows for chaining this configuration after constructing the Shooter object,
     * providing flexibility in how the shooter is initialized and configured.
     *
     * @param alliance     The alliance color (BLUE or RED) to determine which goal to target
     * @param startingPose The initial pose of the robot, used to calculate the initial distance and angle to the goal for setting the initial smoothed values in the ShooterController
     * @return The Shooter instance with the updated shooter model based on the specified alliance and starting pose, allowing for method chaining.
     */
    @NonNull
    public Shooter setAlliancePose(@NonNull Alliance alliance, @NonNull Pose startingPose) {
        this.alliance = alliance;
        shooterModel = shooterModel.setAlliancePose(alliance, startingPose);
        return this;
    }

    /**
     * Sets the base position of the turret for accurate targeting.
     *
     * @param baseX X-coordinate of the turret base in inches
     * @param baseY Y-coordinate of the turret base in inches
     * @return The Shooter instance with the updated turret base values, allowing for method chaining.
     */
    @NonNull
    public Shooter setTurretBaseValues(double baseX, double baseY) {
        turret.setBaseValues(baseX, baseY);
        return this;
    }

    /**
     * Adjusts the shooter model based on whether the last shot was high or low.
     * This allows for dynamic compensation in future shots.
     */
    public void shotWasHigh() {
        shooterModel.adjustShot(true);
    }

    /**
     * Adjusts the shooter model based on whether the last shot was high or low.
     * This allows for dynamic compensation in future shots.
     */
    public void shotWasLow() {
        shooterModel.adjustShot(false);
    }

    /**
     * Returns the current position of the turret in encoder ticks.
     *
     * @return Current turret position in ticks
     */
    public int getTurretCurrentPosition() {
        return turret.getCurrentPosition();
    }

    /**
     * Increases the hood position by a specified increment. The increment is added to the current
     * hood position, and the resulting position is set as the new hood position. The method ensures
     * that the hood position remains within the valid range of 0.0 to 1.0.
     *
     * @param increment Amount to increase the hood position by (positive value)
     */
    public void increaseHoodPosition(double increment) {
        setHoodPosition(getHoodPosition() + increment);
    }

    /**
     * Returns the current position of the hood as a normalized value (0.0 to 1.0).
     * 0.0 corresponds to the lowest hood position, and 1.0 corresponds to the highest hood position.
     *
     * @return Current hood position
     */
    public double getHoodPosition() {
        return hood.getPosition();
    }

    /**
     * Sets the position of the hood using a normalized value (0.0 to 1.0).
     * 0.0 corresponds to the lowest hood position, and 1.0 corresponds to the highest hood position.
     *
     * @param position Desired hood position (0.0 to 1.0)
     */
    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    /**
     * Decreases the hood position by a specified increment. The increment is subtracted from the current
     * hood position, and the resulting position is set as the new hood position. The method ensures
     * that the hood position remains within the valid range of 0.0 to 1.0.
     *
     * @param increment Amount to decrease the hood position by (positive value)
     */
    public void decreaseHoodPosition(double increment) {
        setHoodPosition(getHoodPosition() - increment);
    }

    /**
     * Returns the current RPM of the flywheel.
     *
     * @return Current flywheel RPM
     */
    public double getFlywheelRPM() {
        return flywheel.getRPM();
    }

    /**
     * Sets the target RPM of the flywheel.
     *
     * @param rpm Desired flywheel RPM
     */
    public void setFlywheelRPM(double rpm) {
        flywheel.setRPM(rpm);
    }

    /**
     * Checks if the shooter is ready to shoot by verifying that both the turret is in position and
     * the flywheel is at its target RPM.
     *
     * @return True if the shooter is ready to shoot, false otherwise
     */
    public boolean isReadyToShoot() {
        return isTurretInPosition() && isFlywheelAtTargetRPM();
    }

    /**
     * Checks if the turret is currently at its target position.
     * This method can be used to determine if the turret has finished moving to its desired position
     *
     * @return True if the turret is at its target position, false otherwise
     */
    public boolean isTurretInPosition() {
        return turret.isAtTarget();
    }

    /**
     * Checks if the flywheel is currently at its target RPM.
     * This method can be used to determine if the flywheel has reached the desired speed for shooting.
     *
     * @return True if the flywheel is at its target RPM, false otherwise
     */
    public boolean isFlywheelAtTargetRPM() {
        return flywheel.atTargetRPM();
    }

    /**
     * Sets the target position of the turret in encoder ticks. This method allows for precise control
     * of the turret's position, enabling it to aim accurately at the target.
     *
     * @param position Desired turret position in encoder ticks
     */
    public void setTurretTargetPosition(int position) {
        turret.setTargetPosition(position);
    }

    /**
     * Sets the target position of the turret based on the desired x and y coordinates of the
     * target relative to the robot, and the robot's current heading. This method calculates the
     * necessary turret angle to aim at the target position and sets the turret's target position
     * accordingly. The heading parameter allows for compensation based on the robot's orientation,
     * ensuring accurate aiming even when the robot is not facing directly towards the target.
     *
     * @param x       X-coordinate of the target position relative to the robot in inches
     * @param y       Y-coordinate of the target position relative to the robot in inches
     * @param heading Current heading of the robot in radians, used for calculating the turret
     *                angle to aim at the target
     */
    public void setTurretTargetPosition(double x, double y, double heading) {
        turret.setTargetPosition(x, y, heading);
    }

    @Override
    public void update() {
        // Determine goal based on alliance
        Pose goal = (alliance == Alliance.BLUE) ? BLUE_GOAL_SCORE : RED_GOAL_SCORE;
        Pose pose = localizer.getPose();
        Pose velocity = localizer.getVelocity();

        double heading = pose.getHeading();
        double forwardVel = velocity.getY();
        double lateralVel = velocity.getX();
        double angularVel = velocity.getHeading();

        // Convert robot-relative velocity to field-relative
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double fieldVx = forwardVel * cos - lateralVel * sin;
        double fieldVy = forwardVel * sin + lateralVel * cos;

        // Distance to target
        double dx = goal.getX() - pose.getX();
        double dy = goal.getY() - pose.getY();
        double distanceToTarget = Math.hypot(dx, dy);

        // Shooter LUTs and predictive calculations
        double targetRPM = shooterModel.getFlywheelRPM(distanceToTarget);
        double hoodPosition = shooterModel.getHoodPosition(distanceToTarget);
        double turretLeadAngle = shooterModel.getTurretLeadAngle(
                pose.getX(), pose.getY(), heading,
                fieldVx, fieldVy, angularVel,
                goal.getX(), goal.getY()
        );

        // Apply outputs to hardware
        setFlywheelRPM(targetRPM);
        setHoodPosition(hoodPosition);
        setTurretTargetPosition(dx, dy, heading + turretLeadAngle);
    }
}
