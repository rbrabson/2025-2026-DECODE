package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Hood;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.robotcontrol.ShooterController;

/**
 * Shooter mechanism class for controlling the turret, flywheel, and hood.
 */
public class Shooter implements Mechanism {
    private static final Pose BLUE_GOAL_SCORE = new Pose(25, 18.5, 0);
    private static final Pose RED_GOAL_SCORE = new Pose(25, 125.5, 0);

    private static final double TURRET_DISTANCE_FROM_CENTER = 0; // Inches from the center of the robot to the turret's rotation axis

    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Telemetry telemetry;

    private final ShooterController shooterModel = new ShooterController();

    /**
     * Constructor for the Shooter class.
     *
     * @param hardwareMap The hardware map to access the motors and servos.
     * @param telemetry   The telemetry object for logging data.
     */
    public Shooter(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.turret = new Turret(hardwareMap, telemetry);
        this.flywheel = new Flywheel(hardwareMap, telemetry);
        this.hood = new Hood(hardwareMap, telemetry);
        this.telemetry = telemetry;

        telemetry.addLine("Shooter initialized");
    }

    /**
     * Sets the base values for the turret, which are used as reference points for calculating the
     * turret's target position based on the target's coordinates.
     *
     * @param baseX The x-coordinate of the base position for the turret.
     * @param baseY The y-coordinate of the base position for the turret.
     */
    public void setTurretBaseValues(double baseX, double baseY) {
        turret.setBaseValues(baseX, baseY);
    }

    /**
     * Decreases the RPM of the flywheel motor and hood position by a predefined increment.
     * This is typically used when overshooting artifacts.
     */
    public void shotWasHigh() {
        shooterModel.adjustShot(true);
    }

    /**
     * Increases the RPM of the flywheel motor and hood position by a predefined increment.
     * This is typically used when undershooting artifacts.
     */
    public void shotWasLow() {
        shooterModel.adjustShot(false);
    }


    /**
     * Sets the target position of the turret, adjusting for the offset.
     *
     * @param position The desired target position for the turret, which will be
     *                 adjusted by the TURRET_OFFSET.
     */
    public void setTurretTargetPosition(int position) {
        turret.setTargetPosition(position);
    }

    /**
     * Calculates and sets the turret target position based on the given x, y
     * coordinates and robot heading. The method ensures that the calculated turret
     * position does not exceed the defined left and right limits.
     *
     * @param x       The x-coordinate of the target position.
     * @param y       The y-coordinate of the target position.
     * @param heading The current heading of the robot in radians.
     */
    public void setTurretTargetPosition(double x, double y, double heading) {
        turret.setTargetPosition(x, y, heading);
    }

    /**
     * Returns the current position of the turret motor.
     *
     * @return The current encoder position of the turret motor.
     */
    public int getTurretCurrentPosition() {
        return turret.getCurrentPosition();
    }

    /**
     * Decreases the hood position by a specified increment, ensuring that the new
     * position does not go below the defined minimum.
     *
     * @param increment The amount to decrease the hood position by.
     */
    public void decreaseHoodPosition(double increment) {
        setHoodPosition(getHoodPosition() - increment);
    }

    /**
     * Returns the current position of the hood servo.
     *
     * @return The current position of the hood servo.
     */
    public double getHoodPosition() {
        return hood.getPosition();
    }

    /**
     * Sets the hood position to a specified value, ensuring that it is within the
     * defined minimum and maximum limits.
     *
     * @param position The desired position for the hood, which will be clipped to
     *                 the range defined by HOOD_POSITION_MINIMUM and
     *                 HOOD_POSITION_MAXIMUM.
     */
    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    /**
     * Increases the hood position by a specified increment, ensuring that the new
     * position does not exceed the defined maximum.
     *
     * @param increment The amount to increase the hood position by.
     */
    public void increaseHoodPosition(double increment) {
        setHoodPosition(getHoodPosition() + increment);
    }

    /**
     * Sets the rpm of the flywheel motor to a predefined value for close-range
     * shooting in autonomous mode.
     */
    public void setFlywheelRPMToAutonomousClose() {
        flywheel.autonomousClose();
    }

    /**
     * Sets the rpm of the flywheel motor to a predefined value for far-range
     * shooting in autonomous mode.
     */
    public void setFlywheelRPMToAutonomousFar() {
        flywheel.autonomousFar();
    }

    /**
     * Sets the rpm of the flywheel motor to a predefined value for high-power
     * shooting in teleop mode.
     */
    public void setFlywheelRPMToTeleOpHigh() {
        flywheel.high();
    }

    /**
     * Sets the rpm of the flywheel motor to a predefined value for low-power
     * shooting in teleop mode.
     */
    public void setFlywheelRPMToTeleOpLow() {
        flywheel.low();
    }

    /**
     * Returns the current rpm of the flywheel motor.
     *
     * @return The current rpm of the flywheel motor, obtained using the
     *         getRPM method of the DcMotorEx class.
     */
    public double getFlywheelRPM() {
        return flywheel.getRPM();
    }

    /**
     * Sets the rpm of the flywheel motor to a specified value.
     *
     * @param rpm The desired rpm for the flywheel motor, which will be
     *                 set using the setRPM method of the DcMotorEx class.
     */
    public void setFLywheelRPM(double rpm) {
        flywheel.setRPM(rpm);
    }

    /**
     * Checks if the shooter is ready to shoot, which requires both the turret to be in position
     * and the flywheel to be at the target rpm.
     *
     * @return true if the shooter is ready to shoot, false otherwise.
     */
    public boolean isReadyToShoot() {
        return isTurretInPosition() && isFlywheelAtTargetRPM();
    }

    /**
     * Checks if the turret is currently at its target position.
     *
     * @return true if the turret is at the target position, false otherwise.
     */
    public boolean isTurretInPosition() {
        return turret.isAtTarget();
    }

    /**
     * Checks if the flywheel motor is currently at its target rpm.
     *
     * @return true if the flywheel is at the target rpm, false otherwise.
     */
    public boolean isFlywheelAtTargetRPM() {
        return flywheel.atTargetRPM();
    }

    /**
     * Updates the shooter mechanism based on the robot's current pose and rpm.
     * Calculates the distance to the target and adjusts the flywheel rpm, hood position,
     * and turret target position accordingly. The method uses the ShooterModel to determine the
     * appropriate settings based on the distance and rpm.
     *
     * @param localizer The Localizer object used to obtain the robot's current pose and rpm.
     * @param alliance  The current alliance (RED or BLUE) to determine which goal to target.
     */
    public void update(@NonNull Localizer localizer, @NonNull Alliance alliance) {
        Pose goal = alliance == Alliance.BLUE ? BLUE_GOAL_SCORE : RED_GOAL_SCORE;
        Pose pose = localizer.getPose();
        double dx = pose.getX() - goal.getX();
        double dy = pose.getY() - goal.getY();
        double distanceToTarget = shooterModel.updateDistance(Math.hypot(dx, dy));

        Pose velocity = localizer.getVelocity();
        double forwardVel = velocity.getY();

        double targetRPM = shooterModel.getFlywheelRPM(distanceToTarget, forwardVel);
        double hoodPosition = shooterModel.getHoodPosition(distanceToTarget);
        double turretLeadAngle = shooterModel.getTurretLeadAngle(distanceToTarget, velocity.getX(), TURRET_DISTANCE_FROM_CENTER);
        setFLywheelRPM(targetRPM);
        setHoodPosition(hoodPosition);
        setTurretTargetPosition(dx, dy, pose.getHeading() + turretLeadAngle);
    }

    @Override
    public void update() {
        // NO-OP
    }

}
