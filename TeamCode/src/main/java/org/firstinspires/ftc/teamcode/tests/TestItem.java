package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Abstract class representing a test item for hardware testing. Each test item has a description
 * and a method to run the test.
 */
abstract public class TestItem {
    private final String description;

    /**
     * Constructor for the TestItem class.
     *
     * @param description A brief description of the test item, which will be displayed in the telemetry.
     */
    public TestItem(String description) {
        this.description = description;
    }

    /**
     * Gets the description of the test item.
     *
     * @return The description of the test item.
     */
    public String getDescription() {
        return description;
    }

    /**
     * Abstract method to run the test. Subclasses must implement this method to define the specific
     * behavior of the test when it is executed.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    abstract public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry);
}
