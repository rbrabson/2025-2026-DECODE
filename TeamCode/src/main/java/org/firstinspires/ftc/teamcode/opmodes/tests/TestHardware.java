package org.firstinspires.ftc.teamcode.opmodes.tests;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.tests.TestDriveMotor;
import org.firstinspires.ftc.teamcode.tests.TestFlywheel;
import org.firstinspires.ftc.teamcode.tests.TestHinge;
import org.firstinspires.ftc.teamcode.tests.TestHood;
import org.firstinspires.ftc.teamcode.tests.TestIndexer;
import org.firstinspires.ftc.teamcode.tests.TestIntakeMotor;
import org.firstinspires.ftc.teamcode.tests.TestItem;
import org.firstinspires.ftc.teamcode.tests.TestLight;
import org.firstinspires.ftc.teamcode.tests.TestMecanumDrive;
import org.firstinspires.ftc.teamcode.tests.TestTurret;

import java.util.Arrays;
import java.util.List;

/**
 * A simple teleop opmode for testing individual hardware components. The user can select from a list
 * of tests and run them using the gamepad. This is useful for verifying that hardware components
 * are functioning correctly and can be used for troubleshooting issues with the robot.
 */
@TeleOp(name = "Test Robot Hardware", group = "Tests")
public class TestHardware extends OpMode {
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private List<TestItem> tests;
    private int testNum = 0;
    private boolean runningTest = false;

    /**
     * Initialize the robot and set up the list of tests.
     */
    @Override
    public void init() {
        // Get the motors and servos for testing
        DcMotorEx leftFrontMotor = hardwareMap.get(DcMotorEx.class, "flm");
        DcMotorEx leftRearMotor = hardwareMap.get(DcMotorEx.class, "blm");
        DcMotorEx rightFrontMotor = hardwareMap.get(DcMotorEx.class, "frm");
        DcMotorEx rightRearMotor = hardwareMap.get(DcMotorEx.class, "brm");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "shoot1");
        Servo hinge = hardwareMap.get(Servo.class, "h");
        Servo hood = hardwareMap.get(Servo.class, "s1");
        Servo indexer = hardwareMap.get(Servo.class, "index");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx light = hardwareMap.get(DcMotorEx.class, "l");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");

        // Initialize the motors that require any initialization settings
        initializeDriveMotor(leftFrontMotor, true);
        initializeDriveMotor(leftRearMotor, true);
        initializeDriveMotor(rightFrontMotor, false);
        initializeDriveMotor(rightRearMotor, false);
        initializeFlywheel(flywheel);
        initializeIntakeMotor(intake);
        initializeTurretMotor(turret);

        // Create the test cases
        tests = Arrays.asList(
                new TestFlywheel("Flywheel Motor", flywheel),
                new TestHinge("Hinge Servo", hinge),
                new TestHood("Hood Servo", hood),
                new TestIndexer("Indexer Servo", indexer),
                new TestIntakeMotor("Intake Motor", intake),
                new TestLight("Light", light),
                new TestMecanumDrive("Mecanum Drive", leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor),
                new TestTurret("Turret Motor", turret),
                new TestDriveMotor("> Left Front Motor <", leftFrontMotor),
                new TestDriveMotor("> Left Rear Motor <", leftRearMotor),
                new TestDriveMotor("> Right Front Motor <", rightFrontMotor),
                new TestDriveMotor("> Right Rear Motor <", rightRearMotor)
        );

        telemetry.addLine("Initialization Complete");
    }

    /**
     * Start the OpMode.
     */
    @Override
    public void start() {
        telemetry.addLine("Use Up and Down on D-pad to cycle through choices");
        telemetry.addLine("Use A to start or stop the test");
    }

    /**
     * The main loop of the OpMode. This allows the user to select and run any of the test cases
     * for the robot.
     */
    @Override
    public void loop() {
        // Always get a copy of the current gamepads. The gamepads are updated in near real time,
        // so this ensures that a consistent set of values is used within each mechanism.
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Only select a new test if one isn't currently running, wrapping around the list when
        // the end is reached.
        if (!runningTest) {
            int numTests = tests.size();
            if (currentGamepad1.dpadDownWasPressed()) {
                testNum = (testNum + 1) % numTests;
            } else if (currentGamepad1.dpadUpWasPressed()) {
                testNum = (testNum - 1 + numTests) % numTests;
            }
            telemetry.addData("Test", tests.get(testNum).getDescription());
        }

        // Start or stop the current test when A is pressed.
        if (currentGamepad1.aWasPressed()) {
            runningTest = !runningTest;
            if (runningTest) {
                telemetry.addData("Test", "%s Started", tests.get(testNum).getDescription());
            } else {
                tests.get(testNum).run(false, currentGamepad1, currentGamepad2, telemetry);
                telemetry.addData("Test", "%s Stopped", tests.get(testNum).getDescription());
            }
        }

        // If a test is currently running, execute it with the current gamepad values and telemetry.
        if (runningTest) {
            tests.get(testNum).run(true, currentGamepad1, currentGamepad2, telemetry);
        }

        telemetry.update();
    }

    /**
     * Initialize a drive motor.
     *
     * @param motor   the motor to initialize
     * @param reverse whether to reverse the motor direction
     */
    private void initializeDriveMotor(DcMotorEx motor, boolean reverse) {
        if (reverse) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initialize the flywheel motor with the appropriate direction, PIDF coefficients, and initial
     * velocity.
     *
     * @param motor the flywheel motor to initialize
     */
    private void initializeFlywheel(@NonNull DcMotorEx motor) {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(160, 0, 0, 15));
        motor.setVelocity(0);
    }

    /**
     * Initializes the intake motor settings.
     */
    private void initializeIntakeMotor(@NonNull DcMotorEx motor) {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Initializes the turret motor with the appropriate direction, mode, and
     * initial position.
     */
    private void initializeTurretMotor(@NonNull DcMotorEx motor) {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
}
