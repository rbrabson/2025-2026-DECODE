package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A test item for the intake motor.
 */
public class TestIntakeMotor extends TestItem {
    private final DcMotorEx intakeMotor;

    /**
     * Constructor for the TestItem class.
     *
     * @param description A brief description of the test item, which will be displayed in the telemetry.
     */
    public TestIntakeMotor(String description, DcMotorEx intakeMotor) {
        super(description);
        this.intakeMotor = intakeMotor;
    }

    /**
     * Runs the test for the intake motor. When turned on, the intake motor will start spinning
     * at full power.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param gamepad1  The first gamepad, which can be used for controlling the test if needed.
     * @param gamepad2  The second gamepad, which can also be used for controlling the test if needed.
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        double motorPower = on ? 1.0 : 0.0;
        intakeMotor.setPower(motorPower);
        telemetry.addData(getDescription(), "Power: %.1f", motorPower);
    }
}
