package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.feedforward.FeedForward;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Flywheel subsystem for FTC robot.
 * <p>
 * This class manages a single flywheel motor, providing methods to set target RPMs,
 */
public class Flywheel {
    // Motor encoder
    public static final double TICKS_PER_REV = 537.6;

    // RPM presets
    public static final double RPM_HIGH = 400;
    public static final double RPM_LOW = 300;
    public static final double RPM_AUTON_CLOSE = 150;
    public static final double RPM_AUTON_FAR = 280;
    public static final double RPM_OFF = 0;

    // Tolerance
    public static final double RPM_TOLERANCE = 100;

    // Feedforward gains (example starting values)
    public static final FeedForward FEEDFORWARD = new FeedForward(0.05, 0.00035, 0.00002);

    private final DcMotorEx motor;
    private final FeedForward feedForward;

    @Nullable
    private final Telemetry telemetry;

    private double targetRPM = 0;
    private double lastTargetRPM = 0;
    private long lastUpdateTime;

    /**
     * Constructor for Flywheel subsystem.
     *
     * @param hardwareMap HardwareMap to access motors
     */
    public Flywheel(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for Flywheel subsystem with telemetry.
     *
     * @param hardwareMap HardwareMap to access motors
     * @param telemetry   Telemetry for debugging (can be null)
     */
    public Flywheel(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap);
        this.motor = map.get(DcMotorEx.class, "shoot1");
        this.telemetry = telemetry;
        this.feedForward = FEEDFORWARD;

        initializeMotor();

        lastUpdateTime = System.nanoTime();
    }

    /**
     * Initializes the motor settings, such as direction and mode.
     */
    private void initializeMotor() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // We'll handle velocity control ourselves
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPower(0);
    }

    /**
     * Main control loop for the flywheel. This should be called periodically (e.g., in a loop)
     * to update motor power based on the target RPM and feedforward calculations.
     */
    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        double targetVelocity = rpmToTicksPerSecond(targetRPM);
        double lastVelocity = rpmToTicksPerSecond(lastTargetRPM);

        double acceleration = (targetVelocity - lastVelocity) / dt;

        double ff = feedForward.calculate(0, targetVelocity, acceleration
        );

        // Static friction compensation
        if (targetVelocity != 0) {
            ff += Math.signum(targetVelocity) * feedForward.getKS();
        }

        // Clamp motor power
        double power = Range.clip(ff, -1.0, 1.0);

        motor.setPower(power);

        lastTargetRPM = targetRPM;

        if (telemetry != null) {
            telemetry.addData("Flywheel RPM", getRPM());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Power", power);
        }
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
        targetRPM = rpm;
    }
    /**
     * Converts motor velocity in ticks per second to RPM.
     *
     * @param ticksPerSecond Motor velocity in ticks per second
     * @return Equivalent RPM of the flywheel
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    /**
     * Converts RPM to motor velocity in ticks per second.
     *
     * @param rpm Desired RPM of the flywheel
     * @return Equivalent motor velocity in ticks per second
     */
    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    /**
     * Convenience methods for common RPM presets.
     */
    public void high() {
        setRPM(RPM_HIGH);
    }

    /** Convenience method to set flywheel to low RPM preset.
     */
    public void low() {
        setRPM(RPM_LOW);
    }

    /** Convenience method to set flywheel to autonomous close RPM preset.
     */
    public void autonomousClose() {
        setRPM(RPM_AUTON_CLOSE);
    }

    /**
     * Convenience method to set flywheel to autonomous far RPM preset.
     */
    public void autonomousFar() {
        setRPM(RPM_AUTON_FAR);
    }

    /**
     * Convenience method to stop the flywheel by setting RPM to zero.
     */
    public void stop() {
        setRPM(RPM_OFF);
    }
}
