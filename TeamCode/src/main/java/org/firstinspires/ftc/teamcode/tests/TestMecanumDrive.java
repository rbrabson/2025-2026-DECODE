package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Test class for the mecanum drive system. This class allows for manual control of the
 * mecanum drive using the gamepad's joysticks.
 */
public class TestMecanumDrive extends TestItem {
    protected final DcMotorEx leftFrontMotor;
    protected final DcMotorEx leftRearMotor;
    protected final DcMotorEx rightFrontMotor;
    protected final DcMotorEx rightRearMotor;

    private boolean hasRun = false;

    /**
     * Constructor for the TestMecanumDrive class, which initializes the motor instances for the
     * mecanum drive test.
     *
     * @param description     A brief description of the test item, which will be displayed in the telemetry.
     * @param leftFrontMotor  The DcMotorEx instance representing the left front motor of the mecanum drive.
     * @param leftRearMotor   The DcMotorEx instance representing the left rear motor of the mecanum drive.
     * @param rightFrontMotor The DcMotorEx instance representing the right front motor of the mecanum drive.
     * @param rightRearMotor  The DcMotorEx instance representing
     */
    public TestMecanumDrive(String description, DcMotorEx leftFrontMotor, DcMotorEx leftRearMotor, DcMotorEx rightFrontMotor, DcMotorEx rightRearMotor) {
        super(description);
        this.leftFrontMotor = leftFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightRearMotor = rightRearMotor;
    }

    /**
     * Runs the motor test based on the provided gamepad inputs.
     *
     * @param on        A boolean indicating whether the test should be turned on or off.
     * @param gamepad1  The first gamepad for controlling the motor during the test.
     * @param gamepad2  The second gamepad for controlling the motor during the test (if needed).
     * @param telemetry The Telemetry object for logging test results and status updates.
     */
    @Override
    public void run(boolean on, Gamepad gamepad1, Gamepad gamepad2, @NonNull Telemetry telemetry) {
        if (!on) {
            setPower(0, 0, 0, 0);
            hasRun = false;
            telemetry.addData(getDescription(), "%s Power: %.2f", getDescription(), 0.0);
            return;
        }

        if (!hasRun) {
            telemetry.addData(getDescription(), "Use the joysticks to drive the robot.");
            hasRun = true;
        }

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightRearPower = power * cos / max + turn;

        double maxPower = maxAbs(1.0, leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
        leftFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightFrontPower /= maxPower;
        rightRearPower /= maxPower;

        setPower(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

        telemetry.addData(getDescription(), "%s Power: lfm=%.2, lrm=%2f, rfm=%2f, rrm=%2ff", getDescription(), leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
    }

    /**
     * Helper method to set the power for all four motors of the mecanum drive.
     *
     * @param leftFrontPower  The power to set for the left front motor, typically in the range of -1.0 to 1.0.
     * @param leftRearPower   The power to set for the left rear motor, typically in the range of -1.0 to 1.0.
     * @param rightFrontPower The power to set for the right front motor, typically in the range of -1.0 to 1.0.
     * @param rightRearPower  The power to set for the right rear motor, typically in the range of -1.0 to 1.0.
     */
    private void setPower(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    /**
     * Helper method to calculate the maximum absolute value from a list of values.
     *
     * @param values A variable number of double values to compare.
     * @return The maximum absolute value among the provided values.
     */
    private double maxAbs(@NonNull double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }
}
