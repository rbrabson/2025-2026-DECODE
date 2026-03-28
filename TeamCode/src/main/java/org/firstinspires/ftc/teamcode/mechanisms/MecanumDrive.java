package org.firstinspires.ftc.teamcode.mechanisms;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.pid.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Mode;
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
 *         This mechanism is currently unused. To use it, set TeleOpMode.USE_PEDRO_PATHING to false;
 *     </li>
 * </ol>
 */
public class MecanumDrive implements Drive {
    // --- Constants and Tunable Parameters ---
    private static final double STRAFING_ADJUSTMENT = 1.1;

    // PID coefficients - tune as necessary
    public static final double KP_X = 0.2;
    public static final double KI_X = 0.0;
    public static final double KD_X = 0.0;
    public static final double KP_Y = 0.20;
    public static final double KI_Y = 0.0;
    public static final double KD_Y = 0.0;
    public static final double KP_HEADING = 0.12;
    public static final double KI_HEADING = 0.0;
    public static final double KD_HEADING = 0.10;

    // Completion thresholds for autonomous driving
    private static final double POSE_THRESHOLD = 0.05;      // meters
    private static final double HEADING_THRESHOLD = 0.05;   // radians

    // Static friction compensation (can be tuned)
    private double kStatic = 0.05;

    // Distance scaling factor for autonomous drive (can be tuned)
    private double driveScaling = 0.1;

    // --- Hardware and State ---
    private final DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final Telemetry telemetry;
    private final DriveController driveCtrl;
    private final VoltageCompensator voltageComp;

    private Localizer localizer;

    // PID controllers for autonomous driving
    private final PID pidX;
    private final PID pidY;
    private final PID pidHeading;

    private Pose targetPose = null;
    private boolean fieldCentricEnabled = true;

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

        pidX = new PID(KP_X, KI_X, KD_X);
        pidY = new PID(KP_Y, KI_Y, KD_Y);
        pidHeading = new PID(KP_HEADING, KI_HEADING, KD_HEADING);

        driveCtrl = new DriveController();
        voltageComp = new VoltageCompensator(Robot.getInstance(hardwareMap, telemetry).voltageSensor);

        initializeMotors();

        telemetry.addLine("Mecanum Drive initialized");
    }

    /**
     * Overloaded constructor that allows setting the localizer during initialization. This is
     * useful if you want to create the MecanumDrive and set up the localizer in one step.
     *
     * @param hardwareMap The hardware map to initialize hardware devices.
     * @param localizer   The FusedLocalizer instance to be used for pose estimation.
     * @param telemetry   The telemetry object for logging.
     */
    public MecanumDrive(@NonNull HardwareMap hardwareMap, @NonNull FusedLocalizer localizer, @NonNull Telemetry telemetry) {
        this(hardwareMap, telemetry);
        setLocalizer(localizer);
    }

    // --- Motor Initialization and Configuration ---

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

    // --- Localizer and Pose Methods ---

    /**
     * Sets the localizer for the mecanum drive. The localizer is used to provide pose estimates for
     * the robot, which can be used for field-centric control and autonomous navigation.
     *
     * @param localizer the FusionLocalizer instance to be used for pose estimation.
     * @return the MecanumDrive instance for method chaining.
     */
    public MecanumDrive setLocalizer(@NonNull Localizer localizer) {
        this.localizer = localizer;
        return this;
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
        return localizer != null ? localizer.getPose() : new Pose(0, 0, 0);
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
        if (localizer != null) {
            localizer.setPose(pose);
        }
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
        if (localizer != null) {
            localizer.setStartPose(pose);
        }
        return this;
    }

    // --- Field-Centric Control ---

    /**
     * Enables field-centric control. When enabled, the robot's movement will be relative to the field
     * rather than the robot's orientation.
     */
    public void enableFieldCentric() {
        fieldCentricEnabled = true;
    }

    /**
     * Disables field-centric control. When disabled, the robot's movement will be relative to the robot's
     * orientation rather than the field.
     */
    public void disableFieldCentric() {
        fieldCentricEnabled = false;
    }

    // --- Manual Drive Control ---

    /**
     * Drives the mecanum robot based on joystick inputs for strafing (x), forward/backward movement (y),
     * and rotation (turn).
     *
     * @param x    the left/right strafe input, expected to be in the range of -1 to 1.
     * @param y    the forward/backward input, expected to be in the range of -1 to 1.
     * @param turn the rotation input, expected to be in the range of -1 to 1, where positive
     *             values indicate clockwise rotation, and negative values indicate
     *             counterclockwise rotation.
     */
    public void drive(double x, double y, double turn) {
        // Null check for localizer
        Pose pose = localizer != null ? localizer.getPose() : new Pose(0, 0, 0);

        // If field-centric control is enabled, transform the joystick inputs from robot-centric to
        // field-centric coordinates based on the current heading of the robot, obtained from the IMU.
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

    // --- Autonomous Drive to Target Pose ---

    /**
     * Drives the robot toward a specified target pose using PID control.
     * This method calculates the required x (strafe), y (forward), and turn (rotation)
     * values to move the robot from its current pose to the target pose, and calls the
     * drive() method to apply these values.
     *<p>
     * Completion criteria: If the robot is within POSE_THRESHOLD meters and HEADING_THRESHOLD radians
     * of the target, autonomous driving stops and PIDs are reset.
     *
     * @param target the target pose to drive toward (x, y, heading).
     */
    @SuppressLint("DefaultLocale")
    public void driveToPose(@NonNull Pose target) {
        Pose current = getPose();

        // --- Calculate errors ---
        double xError = target.getX() - current.getX();
        double yError = target.getY() - current.getY();

        // Wrap heading error to [-pi, pi]
        double headingError = target.getHeading() - current.getHeading();
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        // --- Completion check ---
        double distance = Math.hypot(xError, yError);
        if (distance < POSE_THRESHOLD && Math.abs(headingError) < HEADING_THRESHOLD) {
            // Robot has reached the target pose; stop autonomous driving
            targetPose = null;
            pidX.reset();
            pidY.reset();
            pidHeading.reset();
            if (telemetry != null) {
                telemetry.addData("[DRIVE] Target Pose", "Reached");
            }
            return;
        }

        // --- PID outputs ---
        double x = pidX.calculate(target.getX(), current.getX());
        double y = pidY.calculate(target.getY(), current.getY());
        double turn = pidHeading.calculate(0, -headingError);

        // --- Distance-based scaling (prevents full-speed slamming) ---
        double maxDrivePower = Math.min(1.0, distance * driveScaling);

        x = Range.clip(x, -maxDrivePower, maxDrivePower);
        y = Range.clip(y, -maxDrivePower, maxDrivePower);
        turn = Range.clip(turn, -1.0, 1.0);

        // --- Static friction compensation ---
        if (Math.abs(x) > 0.01) x += Math.signum(x) * kStatic;
        if (Math.abs(y) > 0.01) y += Math.signum(y) * kStatic;
        if (Math.abs(turn) > 0.01) turn += Math.signum(turn) * kStatic;

        // --- Final Range.clip ---
        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        turn = Range.clip(turn, -1, 1);

        drive(x, y, turn);

        targetPose = target;

        // --- Telemetry ---
        if (telemetry != null) {
            telemetry.addData("[DRIVE] PID", String.format("x=%.2f y=%.2f turn=%.2f", x, y, turn));
            telemetry.addData("[DRIVE] Error", String.format(
                    "xErr=%.2f yErr=%.2f hErr=%.2f dist=%.2f",
                    xError, yError, headingError, distance));
            telemetry.addData("[DRIVE] Target Pose", target);
        }
    }

    /**
     * Sets whether the user is currently driving the robot. This can be used to disable certain
     * features when the user is not driving, such as automatic alignment or path following.
     *
     * @param isDriving whether the user is currently driving the robot
     */
    public void userDriving(boolean isDriving) {
        if (isDriving) {
            pidX.reset();
            pidY.reset();
            pidHeading.reset();
            targetPose = null;
        }
    }

    // --- Motor Power Calculation and Setting ---

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
    private void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
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

        if (telemetry != null) {
            telemetry.addData("[DRIVE] Left Front Power", leftFrontPower);
            telemetry.addData("[DRIVE] Left Rear Power", leftRearPower);
            telemetry.addData("[DRIVE] Right Front Power", rightFrontPower);
            telemetry.addData("[DRIVE] Right Rear Power", rightRearPower);
        }
    }

    // --- Update Loop ---

    /**
     * update is called each loop for the MecanumDrive mechanism.
     * Updates the localizer and, if a target pose is set, drives toward it.
     */
    @Override
    public void update() {
        if (localizer != null) {
            localizer.update();
        }
        if (targetPose != null) {
            driveToPose(targetPose);
        }
    }

    /**
     * Sets the mode of the localizer, which can affect how the localizer processes sensor data and
     * estimates the robot's pose. Different modes may be optimized for different phases of the match
     * (e.g., autonomous vs. teleop) or for different types of movement.
     *
     * @param mode The mode to set for the localizer, which can be one of the predefined modes in
     *             the FusedLocalizer.Mode enum (e.g., TELEOP, AUTONOMOUS).
     * @return The PedroPathingDrive instance, allowing for method chaining when configuring the localizer mode.
     */
    public MecanumDrive setMode(Mode mode) {
        if (localizer instanceof FusedLocalizer) {
            ((FusedLocalizer) localizer).withMode(mode);
        }
        return this;
    }

    // --- Tuning Methods (Optional for Dashboard) ---

    /**
     * Sets the static friction compensation value.
     * Useful for tuning via dashboard.
     *
     * @param kStatic the new static friction compensation value.
     */
    public void setStaticFriction(double kStatic) {
        this.kStatic = kStatic;
    }

    /**
     * Sets the drive scaling factor for autonomous drive.
     * Useful for tuning via dashboard.
     *
     * @param scaling the new drive scaling factor.
     */
    public void setDriveScaling(double scaling) {
        this.driveScaling = scaling;
    }

    // --- String Representation ---

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
                "MecanumDrive[frontLeftMotor=%s, backLeftMotor=%s, frontRightMotor=%s, backRightMotor=%s, fieldCentricEnabled=%b, pose=%s]",
                frontLeftMotor.getDeviceName(),
                backLeftMotor.getDeviceName(),
                frontRightMotor.getDeviceName(),
                backRightMotor.getDeviceName(),
                fieldCentricEnabled,
                (localizer != null ? localizer.getPose() : "null")
        );
    }
}
