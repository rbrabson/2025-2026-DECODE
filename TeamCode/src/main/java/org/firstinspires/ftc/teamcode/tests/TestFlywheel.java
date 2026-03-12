package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test class for the flywheel mechanism. This class allows you to test the functionality of the
 * flywheel motor by setting it to specific RPM values and displaying the current RPM in telemetry.
 */
public class TestFlywheel extends TestItem {
    private static final double TICKS_PER_REV = 28.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 2000.0;
    private static final double RPM_STEP = 100.0;

    private final DcMotorEx flywheel;

    private double targetRpm = 0.0;

    public TestFlywheel(String description, DcMotorEx flywheel) {
        super(description);
        this.flywheel = flywheel;
        this.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.flywheel.setVelocity(0.0);
    }

    /**
     * Runs the test for the flywheel mechanism. When turned on, the user can control the target RPM
     * using the dpad up and down buttons on gamepad1, and reset to zero RPM using the A button.
     *
     * @param active    A boolean indicating whether the test should be turned on or off    .
     * @param gamepad1  The first gamepad, which can be used for controlling the test if needed.
     * @param gamepad2  The second gamepad, which can also be used for controlling the test if needed.
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean active, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        if (!active) {
            targetRpm = 0.0;
            flywheel.setVelocity(0.0);
            telemetry.addData("Flywheel", "Stopped");
            return;
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetRpm = Range.clip(targetRpm + RPM_STEP, MIN_RPM, MAX_RPM);
        } else if (gamepad1.dpadDownWasPressed()) {
            targetRpm = Range.clip(targetRpm - RPM_STEP, MIN_RPM, MAX_RPM);
        }

        if (gamepad1.aWasPressed()) {
            targetRpm = 0.0;
        }

        double targetTicksPerSec = (targetRpm / 60.0) * TICKS_PER_REV;
        flywheel.setVelocity(targetTicksPerSec);

        double measuredTicksPerSec = flywheel.getVelocity();
        double measuredRpm = (measuredTicksPerSec / TICKS_PER_REV) * 60.0;

        telemetry.addData("Flywheel Controls", "DPad Up/Down: +/-RPM, A: zero");
        telemetry.addData("Target RPM", "%.1f", targetRpm);
        telemetry.addData("Measured RPM", "%.1f", measuredRpm);
        telemetry.addData("Target ticks/s", "%.1f", targetTicksPerSec);
    }
}
