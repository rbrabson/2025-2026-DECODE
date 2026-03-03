package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Tests each drive motor individually.
 *
 * This test lets you run each of the 4 mecanum drive motors one at a time
 * to verify they are plugged in correctly, spinning in the right direction,
 * and responding to commands.
 *
 * CONTROLS:
 *   DPAD UP    - Run front left motor
 *   DPAD DOWN  - Run back left motor
 *   DPAD LEFT  - Run front right motor (no, wait, see below)
 *   DPAD RIGHT - Run back right motor
 *   LEFT STICK Y - Control selected motor speed
 *   A/B/X/Y    - Select individual motor (A=FL, B=FR, X=BL, Y=BR)
 *
 * TODO: [Student] When you spin the front-left motor forward, which wheel
 *       actually moves? Which direction does it spin? If it spins the wrong
 *       way, what needs to change in RobotHardware?
 */
public class DriveMotorTest implements BaseTest {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private int selectedMotor = 0; // 0=FL, 1=FR, 2=BL, 3=BR
    private final String[] motorNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

    @Override
    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "flm");
        frontRight = hardwareMap.get(DcMotor.class, "frm");
        backLeft = hardwareMap.get(DcMotor.class, "blm");
        backRight = hardwareMap.get(DcMotor.class, "brm");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Select motor
        if (gamepad.a) selectedMotor = 0; // Front Left
        if (gamepad.b) selectedMotor = 1; // Front Right
        if (gamepad.x) selectedMotor = 2; // Back Left
        if (gamepad.y) selectedMotor = 3; // Back Right

        // Control selected motor with left stick
        double power = -gamepad.left_stick_y;

        // Reset all motors first
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Power the selected motor
        switch (selectedMotor) {
            case 0: frontLeft.setPower(power); break;
            case 1: frontRight.setPower(power); break;
            case 2: backLeft.setPower(power); break;
            case 3: backRight.setPower(power); break;
        }

        // Telemetry
        telemetry.addLine("=== Drive Motor Test ===");
        telemetry.addLine("A=FL  B=FR  X=BL  Y=BR");
        telemetry.addLine("");
        telemetry.addData("Selected", motorNames[selectedMotor]);
        telemetry.addData("Power (left stick Y)", String.format("%.2f", power));
        telemetry.addLine("");
        telemetry.addData("FL Encoder", frontLeft.getCurrentPosition());
        telemetry.addData("FR Encoder", frontRight.getCurrentPosition());
        telemetry.addData("BL Encoder", backLeft.getCurrentPosition());
        telemetry.addData("BR Encoder", backRight.getCurrentPosition());
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
