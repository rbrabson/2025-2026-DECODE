package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A test for the indexer servo.
 */
public class TestIndexer extends TestItem {
    private final Servo indexer;
    private boolean hasRun = false;
    private double position = 0;

    /**
     * Create a new TestIndexer with the given description and servo.
     *
     * @param description a description of the test
     * @param indexer     the indexer servo to test
     */
    public TestIndexer(String description, Servo indexer) {
        super(description);
        this.indexer = indexer;
    }

    /**
     * Run the test by setting the indexer servo to a specific position based on the value of `on`.
     * When `on` is true, the test allows the user to adjust the indexer position using the dpad left
     * and right buttons on the first gamepad.
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
                telemetry.addLine("Use the dpad left and right buttons to control the indexer position.");
                hasRun = true;
            }
            if (gamepad1.dpad_left) {
                position += 0.05;
            } else if (gamepad1.dpad_right) {
                position -= 0.05;
            }
        } else {
            position = 0;
            hasRun = false;
        }

        position = Math.min(1.0, Math.max(0.0, position));
        indexer.setPosition(position);
        telemetry.addData("Indexer Position", position);
    }
}
