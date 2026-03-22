package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.feedforward.FeedForward;
import com.rbrabson.control.pid.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Flywheel subsystem for FTC robot.
 * <p>
 * This class manages a single flywheel motor, providing methods to set target RPMs,
 * update motor power using feedforward control, and monitor flywheel performance.
 */
public class Flywheel {
    // Motor encoder
    public static final double TICKS_PER_REV = 384;
    public static final double RPM_MAX = 435;

    public static final double RPM_OFF = 0;

    private static final double PID_RESET_THRESHOLD = RPM_MAX / 4.0;

    // Tolerance for determining if flywheel is at target RPM
    public static final double RPM_TOLERANCE = 100;

    // Velocity PID constants (these need to be tuned for your specific robot)
    public static final double kP = 0.00012;
    public static final double kI = 0;
    public static final double kD = 0;

    // Feedforward gains (these need to be tuned for your specific robot)
    public static final double kS = 0.05;
    public static final double kV = 0.00035;
    public static final double kA = 0.00002;

    private final DcMotorEx motor;

    private final Telemetry telemetry;

    private double targetRPM = 0;
    private double lastTargetRPM = 0;
    private long lastUpdateTime;
    private PID velocityPID;
    private FeedForward feedForward;
    private boolean usePID = true;

    // Use class variables for possible telemetry display of feedforward and PID contributions
    private double ff = 0;
    private double pidOutput = 0;

    /**
     * Constructor for Flywheel subsystem with telemetry.
     *
     * @param hardwareMap HardwareMap to access motors
     * @param telemetry   Telemetry for debugging
     */
    public Flywheel(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap);
        this.motor = map.get(DcMotorEx.class, "shoot1");
        this.telemetry = Objects.requireNonNull(telemetry);

        velocityPID = new PID(kP, kI, kD).withOutputLimits(-1.0, 1.0);
        feedForward = new FeedForward(kS, kV, kA);

        initializeMotor();
        lastUpdateTime = System.nanoTime();
    }

    /**
     * Initializes the motor settings, such as direction and mode.
     */
    private void initializeMotor() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    /**
     * Main control loop for the flywheel. Call periodically (e.g., in loop()).
     * Combines feedforward with a velocity PID for improved accuracy.
     */
    public void update() {
        // Reset PID on large RPM changes to prevent windup
        if (Math.abs(targetRPM - lastTargetRPM) > PID_RESET_THRESHOLD) {
            velocityPID.reset();
        }

        long now = System.nanoTime();
        // Clamp dt to prevent spikes from lag
        double dt = Range.clip((now - lastUpdateTime) / 1e9, 1e-3, 0.05);
        lastUpdateTime = now;

        double power = calculatePower(targetRPM, dt);
        motor.setPower(power);

        lastTargetRPM = targetRPM;

        telemetry.addData("Flywheel RPM", getRPM());
        telemetry.addData("Flywheel Target RPM", targetRPM);
        telemetry.addData("Flywheel Feedforward", ff);
        telemetry.addData("Flywheel PID Output", pidOutput);
        telemetry.addData("Flywheel Total Power", power);
        telemetry.addData("Flywheel RPM Error", targetRPM - getRPM());
    }

    /**
     * Calculates the motor power for a given target RPM and dt.
     *
     * @param rpmTarget Target RPM for the flywheel
     * @param dt        Time delta in seconds since last update
     * @return Clipped motor power [-1, 1]
     */
    private double calculatePower(double rpmTarget, double dt) {
        double targetVelocity = rpmToTicksPerSecond(rpmTarget);
        double lastVelocity = rpmToTicksPerSecond(lastTargetRPM);
        double acceleration = (targetVelocity - lastVelocity) / dt;

        // Feedforward component
        ff = feedForward.calculate(0, targetVelocity, acceleration);

        // Static friction compensation
        if (Math.abs(targetVelocity) > 1) {
            ff += Math.signum(targetVelocity) * feedForward.getKS();
        }

        // PID correction on top of feedforward
        pidOutput = usePID ? velocityPID.calculate(targetVelocity, motor.getVelocity(), dt) : 0;
        return Range.clip(ff + pidOutput, -1.0, 1.0);
    }

    /**
     * Gets the current target RPM for the flywheel.
     *
     * @return Current target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Checks if the flywheel is at the target RPM within a specified tolerance.
     *
     * @return True if at target RPM, false otherwise
     */
    public boolean atTargetRPM() {
        return Math.abs(getRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    /**
     * Gets the current RPM of the flywheel based on motor velocity.
     *
     * @return Current RPM of the flywheel
     */
    public double getRPM() {
        return ticksPerSecondToRPM(motor.getVelocity());
    }

    /**
     * Sets the target RPM for the flywheel. The control loop will attempt to reach this RPM.
     *
     * @param rpm Desired target RPM for the flywheel
     */
    public void setRPM(double rpm) {
        targetRPM = Range.clip(rpm, -RPM_MAX, RPM_MAX);
    }

    /**
     * Converts motor velocity in ticks per second to RPM.
     *
     * @param ticksPerSecond Motor velocity in ticks per second
     * @return Equivalent RPM of the flywheel
     */
    private static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    /**
     * Converts RPM to motor velocity in ticks per second.
     *
     * @param rpm Desired RPM of the flywheel
     * @return Equivalent motor velocity in ticks per second
     */
    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    /**
     * Convenience method to stop the flywheel by setting RPM to zero.
     */
    public void stop() {
        setRPM(RPM_OFF);
    }

    /**
     * Gets the current power being applied to the flywheel motor. This can be useful for debugging
     * or telemetry purposes.
     *
     * @return Current motor power being applied to the flywheel
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Enables or disables the use of PID control on top of feedforward. When enabled, the flywheel
     * will use a simple velocity PID to correct for any discrepancies between target
     * and actual RPM.
     *
     * @param usePID True to enable PID control, false to rely solely on feedforward
     */
    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    /**
     * Updates the PID gains at runtime. This resets the PID controller to prevent integral windup.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPIDGains(double kP, double kI, double kD) {
        velocityPID = new PID(kP, kI, kD).withOutputLimits(-1.0, 1.0);
    }

    /**
     * Updates the feedforward gains at runtime. This can be useful for tuning or adapting to changes in
     * the robot's performance over time.
     *
     * @param kS Static gain (voltage needed to overcome static friction)
     * @param kV Velocity gain
     * @param kA Acceleration gain
     */
    public void setFeedForwardConstants(double kS, double kV, double kA) {
        feedForward = new FeedForward(kS, kV, kA);
    }
}
