package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Tests turret range of motion and encoder positions.
 *
 * This test lets you manually move the turret to verify:
 * - The encoder reads correctly (resets to 0 at startup)
 * - The turret moves in the expected direction
 * - The physical limits match the software limits (EXTREME_LEFT=1700, EXTREME_RIGHT=-350)
 * - The RUN_TO_POSITION mode works correctly
 *
 * CONTROLS:
 *   LEFT STICK X   - Move turret left/right (direct power mode)
 *   DPAD LEFT      - Set turret to extreme left position (1700)
 *   DPAD RIGHT     - Set turret to extreme right position (-350)
 *   DPAD UP        - Set turret to center (0)
 *   A              - Toggle between power mode and position mode
 *
 * TODO: [Student] What encoder value does the turret read at 45 degrees?
 *       At 90 degrees? Does it match the expected 742 ticks/90 degrees?
 */
public class TurretTest implements BaseTest {

    private DcMotor turret;
    private boolean positionMode = true;
    private int targetPosition = 0;

    @Override
    public void init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Toggle mode
        if (gamepad.a) {
            positionMode = !positionMode;
            if (positionMode) {
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
            } else {
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (positionMode) {
            // Preset positions
            if (gamepad.dpad_left) targetPosition = RobotHardware.TURRET_EXTREME_LEFT;
            if (gamepad.dpad_right) targetPosition = RobotHardware.TURRET_EXTREME_RIGHT;
            if (gamepad.dpad_up) targetPosition = 0;

            // Manual adjustment with stick
            targetPosition += (int) (gamepad.left_stick_x * 10);

            // Clamp to safe range
            targetPosition = Math.max(RobotHardware.TURRET_EXTREME_RIGHT,
                Math.min(RobotHardware.TURRET_EXTREME_LEFT, targetPosition));

            turret.setTargetPosition(targetPosition);
        } else {
            // Direct power mode
            turret.setPower(gamepad.left_stick_x * 0.5);
        }

        // Telemetry
        telemetry.addLine("=== Turret Test ===");
        telemetry.addData("Mode", positionMode ? "POSITION (A to switch)" : "POWER (A to switch)");
        telemetry.addLine("");
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", turret.getCurrentPosition());
        telemetry.addData("Error", targetPosition - turret.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addData("Extreme Left", RobotHardware.TURRET_EXTREME_LEFT);
        telemetry.addData("Extreme Right", RobotHardware.TURRET_EXTREME_RIGHT);
        telemetry.addData("Ticks/Degree", String.format("%.2f", RobotHardware.TURRET_TICKS_PER_DEGREE));
        telemetry.addData("Estimated Angle", String.format("%.1f deg",
            turret.getCurrentPosition() / RobotHardware.TURRET_TICKS_PER_DEGREE));
    }

    @Override
    public void stop() {
        turret.setPower(0);
    }
}
