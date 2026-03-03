package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Tests the hinge servo that flicks balls into the shooter.
 *
 * The hinge has two key positions:
 *   CLOSED (0.09) - Ball is held in the indexer
 *   OPEN (0.4)    - Ball is released into the flywheel
 *
 * This test lets you toggle between open and closed positions and
 * also fine-tune the positions with the bumpers.
 *
 * CONTROLS:
 *   A             - Set to CLOSED position (0.09)
 *   Y             - Set to OPEN position (0.4)
 *   RIGHT BUMPER  - Increase position by 0.01
 *   LEFT BUMPER   - Decrease position by 0.01
 *   X             - Toggle rapid open-close cycle (simulates shooting)
 *
 * TODO: [Student] Time how long it takes for the hinge to go from closed
 *       to open. Does 175ms (our shortest delay) give it enough time?
 */
public class HingeTest implements BaseTest {

    private Servo hinge;
    private double currentPos = RobotHardware.HINGE_CLOSED;
    private boolean rapidCycle = false;
    private long cycleStartTime = 0;

    @Override
    public void init(HardwareMap hardwareMap) {
        hinge = hardwareMap.get(Servo.class, "h");
        hinge.setPosition(RobotHardware.HINGE_CLOSED);
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Preset positions
        if (gamepad.a) {
            currentPos = RobotHardware.HINGE_CLOSED;
            rapidCycle = false;
        }
        if (gamepad.y) {
            currentPos = RobotHardware.HINGE_OPEN;
            rapidCycle = false;
        }

        // Fine-tune
        if (gamepad.right_bumper) currentPos = Math.min(1.0, currentPos + 0.005);
        if (gamepad.left_bumper) currentPos = Math.max(0.0, currentPos - 0.005);

        // Rapid cycle toggle (simulates shoot sequence)
        if (gamepad.x) {
            rapidCycle = !rapidCycle;
            cycleStartTime = System.currentTimeMillis();
        }

        if (rapidCycle) {
            long elapsed = System.currentTimeMillis() - cycleStartTime;
            long phase = elapsed % 1000; // 1 second cycle
            if (phase < 350) {
                currentPos = RobotHardware.HINGE_OPEN;  // Open for 350ms
            } else {
                currentPos = RobotHardware.HINGE_CLOSED; // Closed for 650ms
            }
        }

        hinge.setPosition(currentPos);

        // Telemetry
        telemetry.addLine("=== Hinge Servo Test ===");
        telemetry.addLine("A=Closed(0.09)  Y=Open(0.4)  X=Rapid Cycle");
        telemetry.addLine("LB/RB to fine-tune");
        telemetry.addLine("");
        telemetry.addData("Current Position", String.format("%.4f", currentPos));
        telemetry.addData("Closed Position", RobotHardware.HINGE_CLOSED);
        telemetry.addData("Open Position", RobotHardware.HINGE_OPEN);
        telemetry.addData("Rapid Cycle", rapidCycle ? "ON" : "OFF");
    }

    @Override
    public void stop() {
        hinge.setPosition(RobotHardware.HINGE_CLOSED);
    }
}
