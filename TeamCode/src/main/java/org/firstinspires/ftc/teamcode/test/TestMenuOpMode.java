package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Scrollable test menu for individually testing robot components.
 *
 * This TeleOp displays a list of available tests on the Driver Station.
 * Use the gamepad to scroll through and select a test to run.
 *
 * CONTROLS:
 *   DPAD UP    - Scroll up in the menu
 *   DPAD DOWN  - Scroll down in the menu
 *   A BUTTON   - Select and run the highlighted test
 *   B BUTTON   - Exit the current test and return to the menu
 *
 * HOW TO ADD A NEW TEST:
 *   1. Create a class that implements {@link BaseTest}
 *   2. Add the test name to the testNames[] array below
 *   3. Add a case for it in the createTest() method
 *   4. The test will automatically appear in the menu
 *
 * TODO: [Student] Can you add a test for the hood extension servo?
 *       What about a test that checks all motors at once?
 */
@TeleOp(name = "Test Menu", group = "Test")
public class TestMenuOpMode extends OpMode {

    // -----------------------------------------------------------------------
    // Menu configuration
    // -----------------------------------------------------------------------

    /** Names of all available tests displayed in the menu. */
    private final String[] testNames = {
        "Drive Motors",
        "Turret Range",
        "Shooter Velocity",
        "Indexer Positions",
        "Hinge Servo",
        "Intake Motor",
        "Limelight Detection",
        "Color Sensor",
        "Full Shoot Cycle",
    };

    /** Currently highlighted menu item index. */
    private int selectedIndex = 0;

    /** Whether we're currently inside a test (vs. the menu). */
    private boolean inTest = false;

    /** The currently running test instance, or null if in the menu. */
    private BaseTest currentTest = null;

    /** Debounce timer to prevent rapid menu scrolling. */
    private final ElapsedTime debounce = new ElapsedTime();

    /** Minimum milliseconds between menu navigation inputs. */
    private static final double DEBOUNCE_MS = 250;

    // -----------------------------------------------------------------------
    // OpMode lifecycle
    // -----------------------------------------------------------------------

    @Override
    public void init() {
        telemetry.addLine("=== TaigaBots 18190 Test Menu ===");
        telemetry.addLine("");
        telemetry.addLine("DPad Up/Down to scroll");
        telemetry.addLine("A to select, B to go back");
        telemetry.update();
    }

    @Override
    public void loop() {
        // If we're inside a test, delegate to the test's loop
        if (inTest && currentTest != null) {
            // Check for exit (B button)
            if (gamepad1.b) {
                currentTest.stop();
                inTest = false;
                currentTest = null;
                debounce.reset();
                return;
            }

            // Run the test
            currentTest.loop(telemetry, gamepad1, hardwareMap);
            telemetry.addLine("");
            telemetry.addLine("Press B to return to menu");
            telemetry.update();
            return;
        }

        // -- Menu Navigation --

        // Scroll up
        if (gamepad1.dpad_up && debounce.milliseconds() > DEBOUNCE_MS) {
            selectedIndex = Math.max(0, selectedIndex - 1);
            debounce.reset();
        }

        // Scroll down
        if (gamepad1.dpad_down && debounce.milliseconds() > DEBOUNCE_MS) {
            selectedIndex = Math.min(testNames.length - 1, selectedIndex + 1);
            debounce.reset();
        }

        // Select test
        if (gamepad1.a && debounce.milliseconds() > DEBOUNCE_MS) {
            currentTest = createTest(selectedIndex);
            if (currentTest != null) {
                currentTest.init(hardwareMap);
                inTest = true;
            }
            debounce.reset();
            return;
        }

        // -- Draw Menu --
        telemetry.addLine("=== TaigaBots 18190 Test Menu ===");
        telemetry.addLine("");

        for (int i = 0; i < testNames.length; i++) {
            // Show ">>" arrow next to the selected item
            String prefix = (i == selectedIndex) ? ">> " : "   ";
            telemetry.addLine(prefix + testNames[i]);
        }

        telemetry.addLine("");
        telemetry.addLine("DPad Up/Down to scroll, A to select");
        telemetry.update();
    }

    // -----------------------------------------------------------------------
    // Test factory
    // -----------------------------------------------------------------------

    /**
     * Creates a test instance based on the menu selection index.
     *
     * @param index  the index into testNames[]
     * @return a new BaseTest instance, or null if index is invalid
     */
    private BaseTest createTest(int index) {
        switch (index) {
            case 0: return new DriveMotorTest();
            case 1: return new TurretTest();
            case 2: return new ShooterTest();
            case 3: return new IndexerTest();
            case 4: return new HingeTest();
            case 5: return new IntakeTest();
            case 6: return new LimelightTest();
            case 7: return new ColorSensorTest();
            case 8: return new FullShootCycleTest();
            default: return null;
        }
    }
}
