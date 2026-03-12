package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TestDriveMotor is a test item that allows the user to control a specific drive motor using
 * the gamepad joysticks. This can be useful when testing individual drive train motors.
 */
public class TestDriveMotor extends TestItem {
    protected final DcMotorEx motor;

    private boolean hasRun = false;

    /**
     * Constructor for the TestItem class.
     *
     * @param description A brief description of the test item, which will be displayed in the telemetry.
     * @param motor       The DcMotorEx instance representing the motor to be tested.
     */
    public TestDriveMotor(@NonNull String description, @NonNull DcMotorEx motor) {
        super(description);
        this.motor = motor;
    }

    /**
     * Runs the motor test based on the provided gamepad inputs.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param gamepad1  The first gamepad for controlling the motor during the test.
     * @param gamepad2  The second gamepad for controlling the motor during the test (if needed).
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        double power;
        if (on) {
            if (!hasRun) {
                telemetry.addLine("Use the left joystick's Y-axis to control the motor's power.");
                hasRun = true;
            }
            power = -gamepad1.left_stick_y;
        } else {
            power = 0;
            hasRun = false;
        }
        power = Math.max(-1, Math.min(1, power));
        motor.setPower(power);
        telemetry.addData(getDescription(), "%s Power: %.2f", getDescription(), power);
    }

    /**
     * Helper method to calculate the maximum absolute value from a list of values.
     *
     * @param values A variable number of double values to compare.
     * @return The maximum absolute value among the provided values.
     */
    private double maxAbs(@NonNull double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }
}
