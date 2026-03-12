package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test class for the turret mechanism. This class allows for manual control of the
 * turret using the gamepad's dpad left and right buttons.
 */
public class TestTurret extends TestItem {
    private static final int MAX_LEFT = 1700;
    private static final int MAX_RIGHT = -350;

    private final DcMotorEx turret;
    private int position = 0;
    private boolean hasRun = false;

    /**
     * Constructor for the TestItem class.
     *
     * @param description A brief description of the test item, which will be displayed in the telemetry.
     */
    public TestTurret(String description, DcMotorEx turret) {
        super(description);
        this.turret = turret;
    }

    /**
     * Runs the test item, allowing for manual control of the turret position using the gamepad's
     * dpad left and right buttons.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param gamepad1  The first gamepad for controlling the turret position.
     * @param gamepad2  The second gamepad (not used in this test).
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        if (on) {
            if (!hasRun) {
                position = 0;
                telemetry.addLine("Use the dpad left and right buttons to control the turret position.");
                hasRun = true;
            }
            if (gamepad1.dpad_left) {
                position += 10;
            } else if (gamepad1.dpad_right) {
                position -= 10;
            }
        } else {
            position = 0;
            hasRun = false;
        }

        position = Math.max(MAX_RIGHT, Math.min(MAX_LEFT, position));
        turret.setTargetPosition(position);
        telemetry.addData("Turret Position", position);
    }
}
