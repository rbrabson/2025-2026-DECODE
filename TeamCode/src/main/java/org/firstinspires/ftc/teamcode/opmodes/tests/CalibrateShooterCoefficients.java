package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.sensors.Limelight;

import java.util.Locale;

/**
 * OpMode for interactively calibrating shooter flywheel velocity and hood
 * position coefficients.
 * <p>
 * Allows the user to select alliance, adjust flywheel velocity, hood position,
 * and simulated distance to goal. The current configuration is displayed as a
 * CSV for easy data collection.
 * <p>
 * Controls:
 * <ul>
 * <li>Dpad Left/Right: Decrease/Increase flywheel velocity</li>
 * <li>Dpad Up/Down: Raise/Lower hood position</li>
 * <li>LB/RB: Decrease/Increase distance to goal</li>
 * <li>X/B: Select Blue/Red alliance and reset pose</li>
 * <li>Options: Snapshot (for future use)</li>
 * </ul>
 */
@TeleOp(name = "Calibrate Shooter Coefficients", group = "Tests")
public class CalibrateShooterCoefficients extends OpMode {

    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String HOOD_NAME = "hood";

    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 4000.0;
    private static final double VELOCITY_STEP = 25.0;

    private static final double MIN_HOOD = 0.0;
    private static final double MAX_HOOD = 1.0;
    private static final double HOOD_STEP = 0.01;

    private static final double MIN_DISTANCE_IN = 0.0;
    private static final double MAX_DISTANCE_IN = 300.0;
    private static final double DISTANCE_STEP_IN = 1.0;

    private static final Pose BLUE_GOAL = new Pose(0, 72, Math.PI / 2.0);
    private static final Pose RED_GOAL = new Pose(144, 72, Math.PI / 2.0);

    private final Gamepad prevGamepad1 = new Gamepad();

    private DcMotorEx flywheel;
    private Servo hood;
    private Limelight limelight;
    private Alliance selectedAlliance = null;

    private double targetVelocity = 0.0;
    private double hoodPos = 0.50;
    private double distanceIn = 72.0;

    /**
     * Initializes hardware and telemetry for shooter calibration. Sets up flywheel,
     * hood, and localizer. Displays control instructions.
     */
    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        limelight = new Limelight(hardwareMap, telemetry);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocity(0.0);
        hood.setPosition(hoodPos);

        telemetry.addLine("Dpad L/R: Velocity, Dpad U/D: Hood");
        telemetry.addLine("X: Blue, B: Red");
        telemetry.addLine("LB/RB: Distance -/+");
        telemetry.update();
    }

    /**
     * Main OpMode loop. Handles user input for adjusting shooter parameters and
     * alliance selection. Updates telemetry with current values and CSV output for
     * data collection.
     */
    @Override
    public void loop() {
        if (gamepad1.x && !prevGamepad1.x) {
            selectedAlliance = Alliance.BLUE;
            limelight.switchToBlueGoal();
            initializePose();
        }
        if (gamepad1.b && !prevGamepad1.b) {
            selectedAlliance = Alliance.RED;
            limelight.switchToRedGoal();
            initializePose();
        }

        if (gamepad1.right_bumper && !prevGamepad1.right_bumper) {
            distanceIn = Range.clip(distanceIn + DISTANCE_STEP_IN, MIN_DISTANCE_IN, MAX_DISTANCE_IN);
        }
        if (gamepad1.left_bumper && !prevGamepad1.left_bumper) {
            distanceIn = Range.clip(distanceIn - DISTANCE_STEP_IN, MIN_DISTANCE_IN, MAX_DISTANCE_IN);
        }

        if (gamepad1.dpad_right && !prevGamepad1.dpad_right) {
            targetVelocity = Range.clip(targetVelocity + VELOCITY_STEP, MIN_VELOCITY, MAX_VELOCITY);
        }
        if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
            targetVelocity = Range.clip(targetVelocity - VELOCITY_STEP, MIN_VELOCITY, MAX_VELOCITY);
        }

        if (gamepad1.dpad_up && !prevGamepad1.dpad_up) {
            hoodPos = Range.clip(hoodPos + HOOD_STEP, MIN_HOOD, MAX_HOOD);
            hood.setPosition(hoodPos);
        }
        if (gamepad1.dpad_down && !prevGamepad1.dpad_down) {
            hoodPos = Range.clip(hoodPos - HOOD_STEP, MIN_HOOD, MAX_HOOD);
            hood.setPosition(hoodPos);
        }

        flywheel.setVelocity(targetVelocity);
        double measuredVelocity = flywheel.getVelocity();

        String allianceText = selectedAlliance == null ? "NONE" : selectedAlliance.name();
        String csv = String.format(Locale.US, "%s,%.2f,%.2f,%.4f", allianceText, distanceIn, measuredVelocity, hoodPos);

        telemetry.addData("Alliance", allianceText);
        telemetry.addData("Distance (in)", "%.2f", distanceIn);
        telemetry.addData("Target Velocity", "%.2f", targetVelocity);
        telemetry.addData("Measured Velocity", "%.2f", measuredVelocity);
        telemetry.addData("Hood", "%.4f", hoodPos);
        telemetry.addData("CSV", csv);
        telemetry.update();

        prevGamepad1.copy(gamepad1);
    }

    /**
     * Initializes the robot's pose and simulated distance to the goal based on the
     * selected alliance and current position.
     */
    private void initializePose() {
        if (selectedAlliance == null)
            return;

        // Get Limelight vision result directly
        com.qualcomm.hardware.limelightvision.LLResult visionResult = limelight.getSensor().getLatestResult();
        boolean visionValid = false;
        Pose poseToUse = new Pose(0, 0, 0);
        if (visionResult != null && visionResult.isValid() && visionResult.getBotpose() != null) {
            double x = visionResult.getBotpose().getPosition().x * 39.3701;
            double y = visionResult.getBotpose().getPosition().y * 39.3701;
            double heading = visionResult.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
            poseToUse = new Pose(x, y, heading);
            visionValid = true;
        }

        Pose goalPose = selectedAlliance == Alliance.BLUE ? BLUE_GOAL : RED_GOAL;
        distanceIn = Math.hypot(poseToUse.getX() - goalPose.getX(), poseToUse.getY() - goalPose.getY());
        distanceIn = Range.clip(distanceIn, MIN_DISTANCE_IN, MAX_DISTANCE_IN);

        if (visionValid) {
            telemetry.log().add("Pose set from Limelight vision. Alliance: "
                    + selectedAlliance.name()
                    + ", distanceIn="
                    + String.format(Locale.US, "%.2f", distanceIn));
        } else {
            telemetry.log().add("Pose set from odometry. Alliance: " + selectedAlliance.name()
                    + ", distanceIn="
                    + String.format(Locale.US, "%.2f", distanceIn));
        }
    }

    /**
     * Stops the flywheel when the OpMode is stopped.
     */
    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setVelocity(0.0);
        }
    }
}
