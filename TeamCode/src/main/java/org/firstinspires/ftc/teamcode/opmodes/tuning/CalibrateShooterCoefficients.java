package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.sensors.Limelight;

import java.util.Locale;

/**
 * OpMode for interactively calibrating shooter flywheel RPM and hood position coefficients.
 * <p>
 * Allows the user to select alliance, adjust flywheel RPM, hood position,
 * and simulated distance to goal. The current configuration is displayed as a
 * CSV for easy data collection.
 * <p>
 * Controls:
 * <ul>
 * <li>Dpad Left/Right: Decrease/Increase flywheel RPM</li>
 * <li>Dpad Up/Down: Raise/Lower hood position</li>
 * <li>LB/RB: Decrease/Increase distance to goal</li>
 * <li>X/B: Select Blue/Red alliance and reset pose</li>
 * <li>Options: Snapshot (for future use)</li>
 * </ul>
 */
@TeleOp(name = "Calibrate Shooter Coefficients", group = "Tuning")
public class CalibrateShooterCoefficients extends OpMode {

    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String HOOD_NAME = "hood";

    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = Flywheel.RPM_MAX;
    private static final double RPM_STEP = 5.0;

    private static final double MIN_HOOD = 0.0;
    private static final double MAX_HOOD = 1.0;
    private static final double HOOD_STEP = 0.01;

    private static final double MIN_DISTANCE_IN = 0.0;
    private static final double MAX_DISTANCE_IN = 300.0;
    private static final double DISTANCE_STEP_IN = 1.0;

    private static final Pose BLUE_GOAL = new Pose(Alliance.BLUE.getBaseX(), Alliance.BLUE.getBaseY(), Math.PI / 2.0);
    private static final Pose RED_GOAL = new Pose(Alliance.RED.getBaseX(), Alliance.RED.getBaseY(), Math.PI / 2.0);

    private DcMotorEx flywheel;
    private Servo hood;
    private Limelight limelight;
    private Alliance selectedAlliance = null;

    private double targetRPM = 0.0;
    private double hoodPos = 0.0;
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
        limelight.switchToRedGoal(); // Default to red, can be changed by user

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setPower(0);
        hood.setPosition(hoodPos);

        selectedAlliance = Alliance.RED;
        limelight.switchToRedGoal();
        initializePose();

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
        if (gamepad1.xWasPressed()) {
            selectedAlliance = Alliance.BLUE;
            limelight.switchToBlueGoal();
            initializePose();
        }
        if (gamepad1.bWasPressed()) {
            selectedAlliance = Alliance.RED;
            limelight.switchToRedGoal();
            initializePose();
        }

        if (gamepad1.rightBumperWasPressed()) {
            distanceIn = Range.clip(distanceIn + DISTANCE_STEP_IN, MIN_DISTANCE_IN, MAX_DISTANCE_IN);
        }
        if (gamepad1.leftBumperWasPressed()) {
            distanceIn = Range.clip(distanceIn - DISTANCE_STEP_IN, MIN_DISTANCE_IN, MAX_DISTANCE_IN);
        }

        if (gamepad1.dpadRightWasPressed()) {
            targetRPM = Range.clip(targetRPM + RPM_STEP, MIN_RPM, MAX_RPM);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM = Range.clip(targetRPM - RPM_STEP, MIN_RPM, MAX_RPM);
        }

        if (gamepad1.dpadUpWasPressed()) {
            hoodPos = Range.clip(hoodPos + HOOD_STEP, MIN_HOOD, MAX_HOOD);
            hood.setPosition(hoodPos);
        }
        if (gamepad1.dpadDownWasPressed()) {
            hoodPos = Range.clip(hoodPos - HOOD_STEP, MIN_HOOD, MAX_HOOD);
            hood.setPosition(hoodPos);
        }

        flywheel.setVelocity(rpmToTicksPerSecond(targetRPM));

        String allianceText = selectedAlliance == null ? "NONE" : selectedAlliance.name();

        // Calculate error
        double measuredRPM = ticksPerSecondToRPM(flywheel.getVelocity());
        double rpmError = targetRPM - measuredRPM;

        String csv = String.format(Locale.US, "%s,%.2f,%.2f,%.4f", allianceText, distanceIn, measuredRPM, hoodPos);

        telemetry.addData("Alliance", allianceText);
        telemetry.addData("Distance (in)", "%.2f", distanceIn);
        telemetry.addData("Target RPM", "%.2f", targetRPM);
        telemetry.addData("Measured RPM", "%.2f", measuredRPM);
        telemetry.addData("RPM Error", "%.2f", rpmError);
        telemetry.addData("Hood", "%.4f", hoodPos);
        telemetry.addData("CSV", csv);

        telemetry.update();
    }

    /**
     * Initializes the robot's pose and simulated distance to the goal based on the
     * selected alliance and current position.
     */
    private void initializePose() {
        // Get Limelight vision result directly
        LLResult visionResult = limelight.getSensor().getLatestResult();
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

    /**
     * Converts RPM to motor velocity in ticks per second.
     *
     * @param rpm Desired RPM of the flywheel
     * @return Equivalent motor velocity in ticks per second
     */
    private static double rpmToTicksPerSecond(double rpm) {
        return (rpm * Flywheel.TICKS_PER_REV) / 60.0;
    }

    /**
     * Converts motor velocity in ticks per second to RPM.
     *
     * @param ticksPerSecond Motor velocity in ticks per second
     * @return Equivalent RPM of the flywheel
     */
    private static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / Flywheel.TICKS_PER_REV;
    }
}
