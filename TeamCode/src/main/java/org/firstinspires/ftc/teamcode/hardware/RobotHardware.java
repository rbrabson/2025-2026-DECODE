package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/**
 * Centralizes all hardware initialization for the TaigaBots 18190 robot.
 *
 * This class replaces the ~60 lines of hardwareMap.get() calls that were
 * previously copy-pasted into every single OpMode. Now there is ONE place
 * to add, remove, or rename hardware devices.
 *
 * USAGE:
 *   RobotHardware hw = new RobotHardware(hardwareMap);
 *   hw.init();
 *   // Now use hw.turret, hw.shooter, hw.indexer, etc.
 *
 * TODO: [Student] If we add a new motor or sensor to the robot, we only
 *       need to change THIS file. Why is that better than changing 9+ files?
 *
 * TODO: [Student] The hardware names ("flm", "brm", "turret", etc.) must
 *       match EXACTLY what is configured in the Robot Controller app.
 *       What happens if there's a typo?
 */
public class RobotHardware {

    // -----------------------------------------------------------------------
    // Drive Motors (Mecanum)
    // These are also managed by PedroPathing, but we keep references for TeleOp
    // -----------------------------------------------------------------------
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    // -----------------------------------------------------------------------
    // Turret
    // A motor that rotates the shooter assembly to aim at the basket
    // -----------------------------------------------------------------------
    public DcMotor turret;

    /** Encoder position limit: furthest the turret can rotate left. */
    public static final int TURRET_EXTREME_LEFT = 1700;

    /** Encoder position limit: furthest the turret can rotate right. */
    public static final int TURRET_EXTREME_RIGHT = -350;

    /**
     * Conversion factor: encoder ticks per degree of turret rotation.
     * Calculated as: 742 ticks / 90 degrees = 8.244 ticks per degree.
     *
     * TODO: [Student] How was 742 ticks per 90 degrees measured?
     *       What gear ratio does the turret use?
     */
    public static final double TURRET_TICKS_PER_DEGREE = 742.0 / 90.0;

    // -----------------------------------------------------------------------
    // Shooter
    // A flywheel motor that launches balls using velocity control
    // -----------------------------------------------------------------------
    public DcMotorEx shooter;

    /**
     * Target shooter velocity in encoder ticks per second.
     *
     * TODO: [Student] How was 1120 ticks/sec chosen? What happens if
     *       the velocity is too low? Too high?
     */
    public static final double SHOOTER_VELOCITY = 1120;

    // -----------------------------------------------------------------------
    // Servos
    // -----------------------------------------------------------------------

    /** Indexer servo: rotates the ball carousel to align slots with intake/shooter. */
    public Servo indexer;

    /**
     * Hinge servo: flicks balls from the indexer into the shooter flywheel.
     * Position 0.09 = closed (ball held), 0.4 = open (ball released to flywheel).
     */
    public Servo hinge;

    /** Hood extension servo: adjusts the angle of the shooter hood. */
    public Servo hoodExtension;

    /** Hinge closed position (ball held in indexer). */
    public static final double HINGE_CLOSED = 0.09;

    /** Hinge open position (ball released into flywheel). */
    public static final double HINGE_OPEN = 0.4;

    // -----------------------------------------------------------------------
    // Intake
    // A motor that spins wheels to pick up balls from the field
    // -----------------------------------------------------------------------
    public DcMotor intake;

    // -----------------------------------------------------------------------
    // Sensors
    // -----------------------------------------------------------------------

    /**
     * Limelight 3A: a smart camera used for AprilTag detection and tracking.
     * Pipeline 0 = AprilTag fiducial detection (used to detect motif)
     * Pipeline 1 = Target tracking (used after motif is detected)
     *
     * TODO: [Student] What is the difference between pipeline 0 and pipeline 1?
     *       What is being tracked in pipeline 1?
     */
    public Limelight3A limelight;

    /** IMU (Inertial Measurement Unit): provides robot heading for field-centric drive. */
    public IMU imu;

    /**
     * Color sensor processor: uses the webcam to detect ball colors (green vs purple).
     * Runs through the VisionPortal framework.
     */
    public PredominantColorProcessor colorSensor;

    /** Light motor: always on during operation for visibility. */
    public DcMotor light;

    // -----------------------------------------------------------------------
    // Private fields
    // -----------------------------------------------------------------------
    private final HardwareMap hardwareMap;

    /**
     * Creates a RobotHardware instance. Call init() after construction.
     *
     * @param hardwareMap  the FTC hardware map from the OpMode
     */
    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initializes all hardware devices. Call this once in your OpMode's init() method.
     *
     * This sets up:
     * - 4 drive motors (with left side reversed for correct mecanum behavior)
     * - Turret motor (encoder-based position control)
     * - Shooter motor (velocity-controlled flywheel with custom PIDF)
     * - 3 servos (indexer, hinge, hood extension)
     * - Intake motor
     * - IMU, Limelight, and color detection webcam
     *
     * TODO: [Student] What does "STOP_AND_RESET_ENCODER" do for the turret?
     *       Why do we call it before "RUN_TO_POSITION"?
     *       Hint: it sets the current physical position as "zero."
     */
    public void init() {
        // -- Drive Motors --
        frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
        backRightMotor = hardwareMap.get(DcMotor.class, "brm");

        // Reverse left-side motors so positive power = forward for all motors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // -- Light --
        light = hardwareMap.get(DcMotor.class, "l");
        light.setPower(1);

        // -- Turret Motor --
        // The turret uses RUN_TO_POSITION mode so we can set a target angle
        // and the motor controller handles getting there automatically.
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        // -- Shooter Motor --
        // Uses DcMotorEx for velocity control with custom PIDF coefficients.
        // P=160 provides aggressive correction, F=15 is the feedforward term.
        shooter = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(160, 0, 0, 15)
        );

        // -- Servos --
        hoodExtension = hardwareMap.get(Servo.class, "s1");
        indexer = hardwareMap.get(Servo.class, "index");
        hinge = hardwareMap.get(Servo.class, "h");

        // -- Intake Motor --
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // -- IMU --
        // The Rev Hub's orientation must match how it's physically mounted.
        // Logo facing LEFT, USB port facing UP.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        );
        imu.initialize(parameters);

        // -- Limelight 3A --
        // Start on pipeline 0 (AprilTag fiducial detection) for motif identification
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // -- Color Sensor (via webcam) --
        // Uses a PredominantColorProcessor to identify green vs purple balls.
        // The ROI (Region of Interest) is set to look at the area where balls
        // sit in front of the color sensor.
        colorSensor = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, -0.5, 0.4, -0.8))
            .setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.WHITE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.YELLOW,
                PredominantColorProcessor.Swatch.BLUE
            )
            .build();

        VisionPortal portal = new VisionPortal.Builder()
            .addProcessor(colorSensor)
            .setCameraResolution(new Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName.class, "logi"))
            .build();

        // -- Set initial servo positions --
        hinge.setPosition(HINGE_CLOSED);
        hoodExtension.setPosition(0);
    }
}
