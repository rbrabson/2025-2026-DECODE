package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test class for the Hinge mechanism. This class allows you to test the functionality of the hinge servo
 * by setting it to specific positions and displaying the current position in telemetry.
 */
public class TestHinge extends TestItem {
    private final Servo hinge;

    private boolean hasRun = false;

    /**
     * Constructor for the TestItem class.
     *
     * @param description A brief description of the test item, which will be displayed in the telemetry.
     */
    public TestHinge(String description, Servo hinge) {
        super(description);
        this.hinge = hinge;
    }

    /**
     * Runs the test for the hinge mechanism. When turned on, the user can control the hinge
     * position using the dpad up and down buttons on gamepad1.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param gamepad1  The first gamepad, which can be used for controlling the test if needed.
     * @param gamepad2  The second gamepad, which can also be used for controlling the test if needed.
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        double position = 0.09;
        if (on) {
            if (!hasRun) {
                telemetry.addLine("Use the dpad up and down buttons to control the hinge position.");
                position = 0.4;
                hasRun = true;
            }

            if (gamepad1.dpadDownWasPressed()) {
                position = 0.09;
            } else if (gamepad1.dpadUpWasPressed()) {
                position = 0.4;
            }
        } else {
            hasRun = false;
        }
        hinge.setPosition(position);
        telemetry.addData("Hinge Position", position);
    }
}
