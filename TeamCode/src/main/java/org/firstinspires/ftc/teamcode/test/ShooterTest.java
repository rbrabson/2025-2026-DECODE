package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Tests the shooter flywheel motor velocity control.
 *
 * This test lets you:
 * - Spin up the shooter at different velocities
 * - See the actual vs target velocity in real-time
 * - Verify the PIDF tuning is working correctly
 * - Check that the motor reaches and holds the target speed
 *
 * CONTROLS:
 *   RIGHT TRIGGER  - Set shooter to competition velocity (1120 ticks/sec)
 *   LEFT TRIGGER   - Set shooter to high velocity (1500 ticks/sec)
 *   A              - Stop the shooter
 *   LEFT STICK Y   - Manual velocity control (0 to 2000 ticks/sec)
 *
 * TODO: [Student] How long does it take for the shooter to reach target speed
 *       from a standstill? Is the velocity stable once it gets there?
 *       What PIDF values could you adjust to make it spin up faster?
 */
public class ShooterTest implements BaseTest {

    private DcMotorEx shooter;
    private double targetVelocity = 0;

    @Override
    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(160, 0, 0, 15));
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Preset velocities
        if (gamepad.right_trigger > 0.5) {
            targetVelocity = RobotHardware.SHOOTER_VELOCITY; // 1120
        }
        if (gamepad.left_trigger > 0.5) {
            targetVelocity = 1500;
        }
        if (gamepad.a) {
            targetVelocity = 0;
        }

        // Manual control with stick
        if (Math.abs(gamepad.left_stick_y) > 0.1) {
            targetVelocity = -gamepad.left_stick_y * 2000;
            targetVelocity = Math.max(0, targetVelocity);
        }

        shooter.setVelocity(targetVelocity);

        // Telemetry
        double actualVelocity = shooter.getVelocity();
        double error = targetVelocity - actualVelocity;

        telemetry.addLine("=== Shooter Test ===");
        telemetry.addLine("RT=1120  LT=1500  A=Stop  Stick=Manual");
        telemetry.addLine("");
        telemetry.addData("Target Velocity", String.format("%.0f ticks/sec", targetVelocity));
        telemetry.addData("Actual Velocity", String.format("%.0f ticks/sec", actualVelocity));
        telemetry.addData("Error", String.format("%.0f ticks/sec", error));
        telemetry.addData("Error %", String.format("%.1f%%",
            targetVelocity > 0 ? (error / targetVelocity * 100) : 0));
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        shooter.setPower(0);
    }
}
