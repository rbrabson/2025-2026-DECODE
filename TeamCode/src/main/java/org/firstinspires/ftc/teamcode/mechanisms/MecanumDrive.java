package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.robotcontrol.DriveController;
import org.firstinspires.ftc.teamcode.robotcontrol.DriveController.DriveOutput;
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
 * <b>NOTES</b>:
 * <ol>
 *     <li>
 *         You MUST call setLocalizer() with a valid FusionLocalizer instance before using the
 *         drive() method.
 *     </li>
 *     <li>
 *         This mechanism is current unused. To use it, set TeleOpMode.USE_PEDRO_PATHING to false;
 *     </li>
 * </ol>
 */
public class MecanumDrive implements Drive {
    private static final double STRAFING_ADJUSTMENT = 1.1;

    private final DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final Telemetry telemetry;
    private final DriveController driveCtrl;

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

        driveCtrl = new DriveController();
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
     * @return the MecanumDrive instance for method chaining.
     */
    public MecanumDrive setLocalizer(@NonNull FusedLocalizer localizer) {
        this.localizer = localizer;
        return this;
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

    /**
     * getLocalizer returns the localizer instance used by the mecanum drive mechanism for pose estimation.
     *
     * @return the localizer instance used by the mecanum drive mechanism.
     */
    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    /**
     * getPose returns the current pose of the robot as determined by the localizer. This method is
     * used to retrieve the robot's position and orientation on the field, which can be used for
     * field-centric control and autonomous navigation.
     *
     * @return the current pose of the robot, including its position (x, y) and heading (theta).
     */
    @Override
    public Pose getPose() {
        return localizer.getPose();
    }

    /**
     * setPose allows you to manually set the robot's pose in the localizer. This can be useful for
     * resetting the robot's position at the start of a match or after a known event (such as
     * picking up a game element). It is important to ensure that the pose is set accurately to
     * maintain correct localization and control of the robot.
     *
     * @param pose the new pose to set for the robot, including its position (x, y) and heading (theta).
     * @return the MecanumDrive instance for method chaining.
     */
    public MecanumDrive setPose(Pose pose) {
        localizer.setPose(pose);
        return this;
    }

    /**
     * setStartPose allows you to set the initial pose of the robot in the localizer. This is typically
     * called during the initialization phase of the robot to establish the starting position and
     * orientation of the robot on the field. Setting the start pose correctly is crucial for accurate
     * localization and control throughout the match.
     *
     * @param pose the initial pose to set for the robot, including its position (x, y) and heading (theta).
     * @return the MecanumDrive instance for method chaining.
     */
    public MecanumDrive setStartPose(Pose pose) {
        localizer.setStartPose(pose);
        return this;
    }

    /**
     * update is a no-op for the MecanumDrive mechanism,
     */
    @Override
    public void update() {
        localizer.update();
    }

    /**
     * Returns a string representation of the MecanumDrive object,
     * including motor names, field-centric status, and current pose.
     *
     * @return a string describing the current state of the MecanumDrive.
     */
    @NonNull
    @Override
    public String toString() {
        return String.format(
                "MecanumDrive[" +
                        "frontLeftMotor=%s, backLeftMotor=%s, frontRightMotor=%s, backRightMotor=%s, " +
                        "fieldCentricEnabled=%b, pose=%s]",
                frontLeftMotor.getDeviceName(),
                backLeftMotor.getDeviceName(),
                frontRightMotor.getDeviceName(),
                backRightMotor.getDeviceName(),
                fieldCentricEnabled,
                (localizer != null ? localizer.getPose() : "null")
        );
    }

}
