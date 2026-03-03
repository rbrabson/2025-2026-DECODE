package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Interface for individual component tests.
 *
 * Each test class implements this interface and focuses on testing ONE
 * specific subsystem or mechanism on the robot. Tests are launched from
 * the {@link TestMenuOpMode} scrollable menu.
 *
 * HOW TO WRITE A NEW TEST:
 *   1. Create a new class that implements BaseTest
 *   2. In init(), get the hardware device(s) you need from hardwareMap
 *   3. In loop(), read gamepad inputs and control the device
 *   4. In loop(), display telemetry showing the device's current state
 *   5. In stop(), set all motors/servos to safe positions
 *   6. Add your test to TestMenuOpMode.testNames[] and createTest()
 *
 * TODO: [Student] Try creating your own test! Pick a mechanism that doesn't
 *       have a test yet and write one following the pattern above.
 */
public interface BaseTest {

    /**
     * Called once when the test is selected from the menu.
     * Use this to initialize hardware devices.
     *
     * @param hardwareMap  the FTC hardware map for accessing devices
     */
    void init(HardwareMap hardwareMap);

    /**
     * Called repeatedly while the test is running.
     * Use this to read gamepad inputs, control devices, and display telemetry.
     *
     * @param telemetry   for displaying data on the Driver Station
     * @param gamepad     the gamepad for controlling the test
     * @param hardwareMap the FTC hardware map (in case you need it during loop)
     */
    void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap);

    /**
     * Called when the test is exited (B button pressed).
     * Use this to stop motors and set servos to safe positions.
     */
    void stop();
}
