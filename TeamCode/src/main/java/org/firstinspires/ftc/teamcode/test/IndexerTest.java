package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;

/**
 * Tests all 6 indexer servo positions (3 intake + 3 shoot).
 *
 * This test cycles through each of the predefined indexer positions so you
 * can verify that each slot aligns correctly with both the intake and the
 * shooter opening.
 *
 * CONTROLS:
 *   DPAD UP       - Go to next position in the cycle
 *   DPAD DOWN     - Go to previous position in the cycle
 *   A             - Switch between CONFIG_A and CONFIG_B
 *   LEFT BUMPER   - Fine-tune position down (-0.001)
 *   RIGHT BUMPER  - Fine-tune position up (+0.001)
 *
 * Position cycle: Intake1 -> Intake2 -> Intake3 -> Shoot1 -> Shoot2 -> Shoot3
 *
 * TODO: [Student] Which config does our competition robot use?
 *       When you go to each position, does the correct slot line up?
 */
public class IndexerTest implements BaseTest {

    private Servo indexer;
    private IndexerConfig config = IndexerConfig.CONFIG_B;
    private boolean useConfigA = false;
    private int positionIndex = 0;
    private double adjustment = 0;
    private final ElapsedTime debounce = new ElapsedTime();

    private final String[] positionNames = {
        "Intake Slot 1", "Intake Slot 2", "Intake Slot 3",
        "Shoot Slot 1", "Shoot Slot 2", "Shoot Slot 3"
    };

    @Override
    public void init(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(Servo.class, "index");
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Switch config
        if (gamepad.a && debounce.milliseconds() > 300) {
            useConfigA = !useConfigA;
            config = useConfigA ? IndexerConfig.CONFIG_A : IndexerConfig.CONFIG_B;
            adjustment = 0;
            debounce.reset();
        }

        // Cycle positions
        if (gamepad.dpad_up && debounce.milliseconds() > 300) {
            positionIndex = (positionIndex + 1) % 6;
            adjustment = 0;
            debounce.reset();
        }
        if (gamepad.dpad_down && debounce.milliseconds() > 300) {
            positionIndex = (positionIndex - 1 + 6) % 6;
            adjustment = 0;
            debounce.reset();
        }

        // Fine-tune
        if (gamepad.right_bumper) adjustment += 0.0005;
        if (gamepad.left_bumper) adjustment -= 0.0005;

        // Get the base position for current index
        double basePosition;
        switch (positionIndex) {
            case 0: basePosition = config.pos1Intake; break;
            case 1: basePosition = config.pos2Intake; break;
            case 2: basePosition = config.pos3Intake; break;
            case 3: basePosition = config.pos1Shoot; break;
            case 4: basePosition = config.pos2Shoot; break;
            case 5: basePosition = config.pos3Shoot; break;
            default: basePosition = 0.5; break;
        }

        double targetPos = Math.max(0, Math.min(1, basePosition + adjustment));
        indexer.setPosition(targetPos);

        // Telemetry
        telemetry.addLine("=== Indexer Test ===");
        telemetry.addData("Config", useConfigA ? "CONFIG_A (press A to switch)" : "CONFIG_B (press A to switch)");
        telemetry.addLine("DPad Up/Down to cycle positions");
        telemetry.addLine("LB/RB to fine-tune");
        telemetry.addLine("");
        telemetry.addData("Position", positionNames[positionIndex]);
        telemetry.addData("Base Value", String.format("%.4f", basePosition));
        telemetry.addData("Adjustment", String.format("%.4f", adjustment));
        telemetry.addData("Final Position", String.format("%.4f", targetPos));
        telemetry.addData("Increment", String.format("%.4f", config.increment));
    }

    @Override
    public void stop() {
        // Leave indexer at current position (safe)
    }
}
