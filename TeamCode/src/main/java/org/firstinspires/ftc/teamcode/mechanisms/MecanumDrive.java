package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.robotcontrol.MecanumDriveController;
import org.firstinspires.ftc.teamcode.robotcontrol.MecanumDriveController.DriveOutput;
import org.firstinspires.ftc.teamcode.robotcontrol.VoltageCompensator;
import org.firstinspires.ftc.teamcode.pedroPathing.FusedLocalizer;
import org.firstinspires.ftc.teamcode.utils.MathEx;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.Arrays;
import java.util.Collection;
import java.util.Objects;

/**
 * Mechanism class for controlling a mecanum drive system.
 * <p>
 * NOTE: You MUST call setLocalizer() with a valid FusionLocalizer instance before using the drive()
 *       method.
 */
public class MecanumDrive implements Mechanism {
    private static final double STRAFING_ADJUSTMENT = 1.1;

    private final DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final Telemetry telemetry;
    private final MecanumDriveController driveCtrl;

    private final VoltageCompensator voltageComp;
    private FusedLocalizer localizer;

    boolean fieldCentricEnabled = true;

    /**
     * Constructor for the MecanumDrive mechanism.
     *
     * @param hardwareMap The hardware map to initialize hardware devices.
     * @param telemetry   The telemetry object for logging.
     */
    public MecanumDrive(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap);
        frontLeftMotor = map.get(DcMotorEx.class, "flm");
        frontRightMotor = map.get(DcMotorEx.class, "frm");
        backLeftMotor = map.get(DcMotorEx.class, "blm");
        backRightMotor = map.get(DcMotorEx.class, "brm");

        this.telemetry = telemetry;

        driveCtrl = new MecanumDriveController();
        voltageComp = new VoltageCompensator(Robot.getInstance(hardwareMap, telemetry).voltageSensor);

        initializeMotors();

        telemetry.addLine("Mecanum Drive initialized");
    }

    /**
     * Initializes the motor settings for the mecanum drive system, including
     * direction, motor type, zero power behavior, and run mode.
     */
    private void initializeMotors() {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Collection<DcMotorEx> motors = Arrays.asList(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Sets the localizer for the mecanum drive. The localizer is used to provide pose estimates for
     * the robot, which can be used for field-centric control and autonomous navigation.
     *
     * @param localizer the FusionLocalizer instance to be used for pose estimation.
     */
    public void setLocalizer(FusedLocalizer localizer) {
        this.localizer = localizer;
    }

    /**
     * Enables or disables field-centric control. When field-centric control is enabled, the robot's
     * movement will be relative to the field rather than the robot's orientation.
     */
    public void enableFieldCentric() {
        fieldCentricEnabled = true;
    }

    /**
     * Disables field-centric control. When field-centric control is disabled, the robot's movement will
     * be relative to the robot's orientation rather than the field.
     */
    public void disableFieldCentric() {
        fieldCentricEnabled = false;
    }

    /**
     * Drives the mecanum robot based on joystick inputs for strafing (x), forward/backward
     * movement (y),
     *
     * @param x         the left/right strafe input, expected to be in the range of -1 to 1.
     * @param y         the forward/backward input, expected to be in the range of -1 to 1.
     * @param turn the rotation input, expected to be in the range of -1 to 1, where positive
     *                  values indicate clockwise rotation, and negative values indicate
     *                  counterclockwise rotation.
     */
    public void drive(double x, double y, double turn) {
        // Get the robot's heading, adjusted for the initial robot pose
        Pose pose = localizer.getPose();

        // If field-centric control is enabled, transform the joystick inputs from robot-centric to
        // field-centric coordinates based on the current heading of the robot, obtained from the
        // IMU.
        if (fieldCentricEnabled) {
            Vector2 rotated = new Vector2(x, y).rotate(-pose.getHeading());
            x = rotated.x;
            y = rotated.y;
        }

        // Update the heading and turn based on the current pose and driver inputs
        DriveOutput driveValues = driveCtrl.update(x, y, turn, pose);

        double[] powers = mecanumSolve(driveValues.getX(), driveValues.getY(), driveValues.getTurn());
        powers = voltageComp.compensate(powers);
        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    /**
     * Calculates the power for each motor based on the desired x (strafe), y (forward/backward),
     * and turn inputs.
     *
     * @param x    the left/right strafe input, expected to be in the range of -1 to 1.
     * @param y    the forward/backward input, expected to be in the range of -1 to 1.
     * @param turn the rotation input, expected to be in the range of -1 to 1, where positive values indicate
     * @return an array of motor powers in the order: [leftFront, leftRear, rightRear, rightFront].
     */
    private @NonNull double[] mecanumSolve(double x, double y, double turn) {
        x *= STRAFING_ADJUSTMENT;
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double rightRearPower = power * cos / max - turn;

        double scale = Math.max(1, power + Math.abs(turn));

        return new double[]{
                leftFrontPower / scale,
                leftRearPower / scale,
                rightRearPower / scale,
                rightFrontPower / scale
        };
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power
     * of 1.0.
     *
     * @param leftFrontPower  the power for the left front motor.
     * @param leftRearPower   the power for the left rear motor
     * @param rightRearPower  the power for the right rear motor.
     * @param rightFrontPower the power for the right front motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        // Normalize the motor powers so that no value exceeds 1.0
        double maxPower = MathEx.maxAbs(1.0, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);
        leftFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightFrontPower /= maxPower;
        rightRearPower /= maxPower;

        frontLeftMotor.setPower(leftFrontPower);
        backLeftMotor.setPower(leftRearPower);
        frontRightMotor.setPower(rightFrontPower);
        backRightMotor.setPower(rightRearPower);

        telemetry.addData("[DRIVE] Left Front Power", leftFrontPower);
        telemetry.addData("[DRIVE] Left Rear Power", leftRearPower);
        telemetry.addData("[DRIVE] Right Front Power", rightFrontPower);
        telemetry.addData("[DRIVE] Right Rear Power", rightRearPower);
    }

    @Override
    public void update() {
        // NO-OP
    }
}
