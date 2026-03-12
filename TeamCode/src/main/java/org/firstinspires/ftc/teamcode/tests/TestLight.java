package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test class for the light mechanism.
 */
public class TestLight extends TestItem {
    private final DcMotorEx light;
    private final Gamepad previousGamepad1 = new Gamepad();
    private boolean isOn = false;
    private boolean hasRun = false;

    /**
     * Constructor for the TestItem class.
     *
     * @param description A brief description of the test item, which will be displayed in the telemetry.
     * @param light       The DcMotorEx representing the light to be tested.
     */
    public TestLight(String description, DcMotorEx light) {
        super(description);
        this.light = light;
    }

    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        if (on) {
            if (!hasRun) {
                hasRun = true;
                isOn = true;
                previousGamepad1.copy(gamepad1);
                telemetry.addLine("Press the X button to toggle the light on and off.");
            }
            if (gamepad1.x && !previousGamepad1.x) {
                isOn = !isOn;
            }
        } else {
            isOn = false;
            hasRun = false;
        }

        double power = isOn ? 1 : 0;
        light.setPower(power);
        telemetry.addData("Light On", isOn);

        previousGamepad1.copy(gamepad1);
    }
}
