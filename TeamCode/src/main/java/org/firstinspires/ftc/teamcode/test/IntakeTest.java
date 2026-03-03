package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Tests the intake motor.
 *
 * The intake picks up balls from the field and feeds them into the indexer.
 * This test lets you run the intake forward, reverse, and at variable speeds.
 *
 * CONTROLS:
 *   A              - Run intake forward (full power)
 *   B              - Stop intake
 *   X              - Run intake reverse (for un-jamming)
 *   LEFT STICK Y   - Variable speed control
 *
 * TODO: [Student] Does the intake spin in the correct direction to pick up balls?
 *       If not, what do you need to change in RobotHardware?
 */
public class IntakeTest implements BaseTest {

    private DcMotor intake;

    @Override
    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        double power = 0;

        // Preset powers
        if (gamepad.a) power = 1.0;    // Forward full
        if (gamepad.b) power = 0;       // Stop
        if (gamepad.x) power = -1.0;    // Reverse full

        // Variable control overrides presets
        if (Math.abs(gamepad.left_stick_y) > 0.1) {
            power = -gamepad.left_stick_y;
        }

        intake.setPower(power);

        // Telemetry
        telemetry.addLine("=== Intake Motor Test ===");
        telemetry.addLine("A=Forward  B=Stop  X=Reverse  Stick=Variable");
        telemetry.addLine("");
        telemetry.addData("Power", String.format("%.2f", power));
        telemetry.addData("Direction", power > 0 ? "FORWARD (picking up)" :
            power < 0 ? "REVERSE (ejecting)" : "STOPPED");
    }

    @Override
    public void stop() {
        intake.setPower(0);
    }
}
