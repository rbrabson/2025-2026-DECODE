package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShootingStateMachine;

/**
 * End-to-end test of the complete 3-ball shooting cycle.
 *
 * This test:
 * 1. Spins up the shooter flywheel
 * 2. Runs the ShootingStateMachine with a test pattern
 * 3. Shows the state machine progressing through all states
 * 4. Fires all 3 balls in sequence
 *
 * Use this to verify the entire shooting pipeline works correctly
 * before running it in autonomous or teleop.
 *
 * CONTROLS:
 *   A              - Start shooting cycle with pattern "GPP", motif "GPP" (aligned)
 *   X              - Start shooting cycle with pattern "GPP", motif "PGP" (offset)
 *   Y              - Start shooting cycle with pattern "GPP", motif "PPG" (offset)
 *   B              - Emergency stop and reset
 *   RIGHT TRIGGER  - Spin up shooter (hold)
 *   DPAD LEFT      - Use CONFIG_A
 *   DPAD RIGHT     - Use CONFIG_B
 *
 * TODO: [Student] Run all 3 scenarios (A, X, Y) and observe:
 *       - Do the balls shoot in different orders for each motif?
 *       - Does the timing feel right?
 *       - Are all 3 balls ejected successfully?
 */
public class FullShootCycleTest implements BaseTest {

    private DcMotorEx shooter;
    private Servo indexer;
    private Servo hinge;
    private ShootingStateMachine shooterSM;
    private IndexerConfig config = IndexerConfig.CONFIG_B;
    private boolean useConfigA = false;
    private boolean shooterSpinning = false;

    @Override
    public void init(HardwareMap hardwareMap) {
        // Initialize hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(160, 0, 0, 15));

        indexer = hardwareMap.get(Servo.class, "index");
        hinge = hardwareMap.get(Servo.class, "h");
        hinge.setPosition(RobotHardware.HINGE_CLOSED);

        // Create shooting state machine
        shooterSM = new ShootingStateMachine(indexer, hinge, config,
            1000, 1175, 1350);
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Config switching
        if (gamepad.dpad_left) {
            useConfigA = true;
            config = IndexerConfig.CONFIG_A;
            shooterSM = new ShootingStateMachine(indexer, hinge, config,
                500, 675, 850);
        }
        if (gamepad.dpad_right) {
            useConfigA = false;
            config = IndexerConfig.CONFIG_B;
            shooterSM = new ShootingStateMachine(indexer, hinge, config,
                1000, 1175, 1350);
        }

        // Spin up shooter
        if (gamepad.right_trigger > 0.5) {
            shooter.setVelocity(RobotHardware.SHOOTER_VELOCITY);
            shooterSpinning = true;
        } else {
            shooter.setVelocity(0);
            shooterSpinning = false;
        }

        // Start shooting sequences with different motifs
        if (gamepad.a && !shooterSM.isShooting()) {
            shooterSM.startShooting("GPP", "GPP", false);
        }
        if (gamepad.x && !shooterSM.isShooting()) {
            shooterSM.startShooting("GPP", "PGP", false);
        }
        if (gamepad.y && !shooterSM.isShooting()) {
            shooterSM.startShooting("GPP", "PPG", false);
        }

        // Emergency stop
        if (gamepad.b) {
            shooterSM.reset();
            hinge.setPosition(RobotHardware.HINGE_CLOSED);
        }

        // Update state machine
        shooterSM.update(telemetry);

        // Telemetry
        telemetry.addLine("=== Full Shoot Cycle Test ===");
        telemetry.addData("Config", useConfigA ? "CONFIG_A (DPad L/R to switch)" : "CONFIG_B");
        telemetry.addLine("");
        telemetry.addLine("Hold RT to spin up shooter");
        telemetry.addData("Shooter", shooterSpinning
            ? String.format("ON (%.0f ticks/s)", shooter.getVelocity()) : "OFF");
        telemetry.addLine("");
        telemetry.addLine("A = Shoot (motif GPP = aligned)");
        telemetry.addLine("X = Shoot (motif PGP = offset +1)");
        telemetry.addLine("Y = Shoot (motif PPG = offset +2)");
        telemetry.addLine("B = Emergency stop");
        telemetry.addLine("");
        telemetry.addData("State Machine", shooterSM.getState());
        telemetry.addData("Is Shooting", shooterSM.isShooting());
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        shooter.setPower(0);
        hinge.setPosition(RobotHardware.HINGE_CLOSED);
        shooterSM.reset();
    }
}
