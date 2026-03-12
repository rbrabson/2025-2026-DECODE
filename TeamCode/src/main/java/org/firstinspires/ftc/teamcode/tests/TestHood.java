package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test class for the hood mechanism. This class allows us to test the functionality of the hood
 * servo by setting it to specific positions and logging the current position to telemetry.
 */
public class TestHood extends TestItem {
    private static final double MAX_POSITION = 1.0;
    private static final double MIN_POSITION = 0.0;

    private final Servo hood;

    private boolean hasRun = false;
    private double position = 0;

    /**
     * Constructor for the TestHood class, which initializes the hood servo and sets the description
     *
     * @param description A brief description of the test, which will be displayed in telemetry when
     *                    the test is run.
     * @param hood        The Servo object representing the hood mechanism that we want to test.
     */
    public TestHood(String description, Servo hood) {
        super(description);

        this.hood = hood;
    }

    /**
     * Runs the test by controlling the hood servo position based on the input from the gamepads.
     * When the test is turned on, it allows the user to adjust the hood position using the dpad up
     * and down buttons on the first gamepad.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param gamepad1  The first gamepad, which can be used for controlling the test if needed.
     * @param gamepad2  The second gamepad, which can also be used for controlling the test if needed.
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        if (on) {
            if (!hasRun) {
                telemetry.addLine("Use the dpad up and down buttons to control the hood position.");
                position = 0;
                hasRun = true;
            }

            if (gamepad1.dpad_down) {
                position -= 0.05;
            } else if (gamepad1.dpad_up) {
                position -= 0.05;
            }
        } else {
            position = 0;
            hasRun = false;
        }

        position = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        hood.setPosition(position);
        telemetry.addData("Hood Position", position);
    }
}
