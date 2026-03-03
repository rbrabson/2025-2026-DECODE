package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.AllianceConfig;
import org.firstinspires.ftc.teamcode.util.PatternUtil;
import org.firstinspires.ftc.teamcode.util.SharedState;

/**
 * Unified TeleOp for both alliances.
 * Replaces the separate jwb.java (Blue) and jwr.java (Red) files.
 *
 * CONTROLS (Gamepad 1 or Gamepad 2):
 *
 *   LEFT STICK      - Field-centric drive (forward/back, strafe)
 *   RIGHT STICK X   - Rotate robot
 *   OPTIONS          - Reset IMU yaw (recalibrate field-centric heading)
 *
 *   LEFT BUMPER     - Toggle turret tracking on/off
 *   RIGHT BUMPER    - Start automated 3-ball shooting sequence
 *   RIGHT TRIGGER   - Toggle intake on/off (hold > 0.5 for 500ms to toggle)
 *   LEFT TRIGGER    - Force center control (pre-position indexer for shooting)
 *
 *   Y               - Toggle shooter velocity (low: 1170 / high: 1500)
 *   A               - Queue green ball for manual shoot
 *   X               - Queue purple ball for manual shoot
 *
 * MANUAL SHOOTING:
 *   Press A or X to queue specific balls for manual shooting.
 *   Each press adds a ball to the shoot queue. The robot will shoot them
 *   one at a time in the order you queued them.
 *
 * TODO: [Student] Walk through a typical TeleOp sequence:
 *       1. Driver picks up balls using intake (RIGHT TRIGGER)
 *       2. Color sensor detects ball colors automatically
 *       3. When all 3 balls are loaded, indexer pre-positions
 *       4. Driver enables turret tracking (LEFT BUMPER)
 *       5. Driver triggers shooting (RIGHT BUMPER)
 *       6. Shooting state machine fires all 3 balls
 *       7. Pattern resets to "XXX", ready for next cycle
 *
 * TODO: [Student] What is field-centric drive? How is it different from
 *       robot-centric drive? Why do we need the IMU for field-centric?
 */
public abstract class MainTeleOp extends LinearOpMode {

    private final AllianceConfig alliance;

    /**
     * Creates a MainTeleOp for the specified alliance.
     * Subclasses (TeleOpBlue, TeleOpRed) call this with their alliance.
     *
     * @param alliance  BLUE or RED
     */
    protected MainTeleOp(AllianceConfig alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        // -- Initialize hardware --
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.init();

        // -- Initialize PedroPathing follower --
        // Start from the position where autonomous left off
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
            SharedState.xPos, SharedState.yPos, SharedState.yaw));

        // TeleOp starts in tracking mode (pipeline 1) since motif is already detected
        hw.limelight.pipelineSwitch(1);

        // -- Create subsystems --
        IntakeSubsystem intakeSub = new IntakeSubsystem(hw.intake);
        IndexerConfig idxConfig = IndexerConfig.CONFIG_B;
        TurretSubsystem turretSub = new TurretSubsystem(
            hw.turret, hw.limelight, alliance.targetX, alliance.targetY);
        ShootingStateMachine shooterSM = new ShootingStateMachine(
            hw.indexer, hw.hinge, idxConfig, 1000, 1175, 1350);
        IndexerSubsystem indexerSub = new IndexerSubsystem(
            hw.indexer, hw.colorSensor, idxConfig, "XXX");

        // Load motif from autonomous
        String motif = SharedState.motif;
        turretSub.setMotif(motif);

        // -- TeleOp state variables --
        boolean turretTrackingEnabled = false;
        boolean intakeToggle = false;
        boolean highVelocity = false;
        boolean intakeBool = false;

        // Manual shooting state
        String manualShootQueue = "";
        boolean manualShooting = false;
        int manualPosition = 0;
        boolean manualPositionControl = true;
        boolean manualCondition = false;
        boolean manualFlag = true;

        // Debounce timers
        ElapsedTime rightTriggerDebounce = new ElapsedTime();
        ElapsedTime leftTriggerDebounce = new ElapsedTime();
        ElapsedTime intakeDelay = new ElapsedTime();
        ElapsedTime manualIndexerTimer = new ElapsedTime();
        ElapsedTime manualHingeTimer = new ElapsedTime();
        ElapsedTime manualCloseTimer = new ElapsedTime();

        // -- Wait for START --
        waitForStart();
        if (isStopRequested()) return;

        // -- Main TeleOp Loop --
        while (opModeIsActive()) {
            follower.update();

            // =============================================================
            // FIELD-CENTRIC MECANUM DRIVE
            // =============================================================
            double y = -gamepad1.left_stick_y;  // Forward/back (inverted)
            double x = gamepad1.left_stick_x;    // Strafe left/right
            double rx = gamepad1.right_stick_x;  // Rotate

            // Reset IMU yaw when OPTIONS is pressed
            if (gamepad1.options) {
                hw.imu.resetYaw();
            }

            // Get robot heading from IMU for field-centric transformation
            double botHeading = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate joystick inputs by negative heading to make controls field-centric
            // TODO: [Student] This is a 2D rotation matrix. What does it do mathematically?
            //       If the robot is facing right (90 degrees), and you push the stick forward,
            //       the robot should still drive "forward" relative to the field (not right).
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Counteract imperfect strafing
            rotX = rotX * 1.1;

            // Calculate motor powers using mecanum kinematics
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            hw.frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            hw.backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            hw.frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            hw.backRightMotor.setPower((rotY + rotX - rx) / denominator);

            // =============================================================
            // TURRET TRACKING TOGGLE
            // =============================================================
            if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                turretTrackingEnabled = !turretTrackingEnabled;
            }
            if (turretTrackingEnabled) {
                turretSub.update(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getHeading(),
                    telemetry, gamepad1, gamepad2
                );
            }

            // =============================================================
            // SHOOTER VELOCITY TOGGLE
            // =============================================================
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                highVelocity = !highVelocity;
            }
            hw.shooter.setVelocity(highVelocity ? 1500 : 1170);
            telemetry.addData("Shooter", highVelocity ? "HIGH (1500)" : "LOW (1170)");

            // Hood position
            hw.hoodExtension.setPosition(0);

            // =============================================================
            // MANUAL SHOOTING (queue specific balls with A/X buttons)
            // =============================================================
            String pattern = indexerSub.getPattern();
            char green = 'G';
            char purple = 'P';
            char x1 = 'X';

            // Queue balls for manual shooting
            if ((gamepad1.aWasPressed() || gamepad2.aWasPressed())
                && PatternUtil.count(pattern, green) >= 1) {
                manualShootQueue += "G";
            }
            if ((gamepad1.xWasPressed() || gamepad2.xWasPressed())
                && PatternUtil.count(pattern, purple) >= 1) {
                manualShootQueue += "P";
            }

            // Execute manual shooting queue
            if (!manualShootQueue.isEmpty() && !shooterSM.isShooting()) {
                manualShooting = true;

                if (manualPositionControl) {
                    manualPosition = pattern.indexOf(manualShootQueue.charAt(0));
                    manualIndexerTimer.reset();
                    manualHingeTimer.reset();
                    manualCloseTimer.reset();
                    indexerSub.setCenterControl(true);
                    manualPositionControl = false;
                }

                // Move indexer to the ball's slot and shoot it
                hw.indexer.setPosition(idxConfig.getShootPosition(manualPosition));

                if (manualIndexerTimer.milliseconds() > 500) {
                    if (manualFlag) {
                        hw.hinge.setPosition(RobotHardware.HINGE_OPEN);
                    }
                    if (manualHingeTimer.milliseconds() > 675) {
                        hw.hinge.setPosition(RobotHardware.HINGE_CLOSED);
                        manualFlag = false;
                        if (manualCloseTimer.milliseconds() > 850) {
                            manualHingeTimer.reset();
                            manualIndexerTimer.reset();
                            manualCloseTimer.reset();
                            manualFlag = true;
                            manualCondition = true;
                        }
                    }
                }

                // Ball shot successfully - update pattern and dequeue
                if (manualCondition) {
                    manualCondition = false;
                    manualPositionControl = true;
                    indexerSub.setPattern(PatternUtil.replaceAt(
                        indexerSub.getPattern(), manualPosition, 'X'));

                    manualShootQueue = manualShootQueue.substring(1);
                }

                if (manualShootQueue.isEmpty()) {
                    manualShooting = false;
                    indexerSub.setCenterControl(false);
                    intakeBool = true;
                }
            } else {
                manualHingeTimer.reset();
                manualIndexerTimer.reset();
                manualCloseTimer.reset();
            }

            // =============================================================
            // AUTOMATED 3-BALL SHOOTING (RIGHT BUMPER)
            // =============================================================
            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                shooterSM.startShooting(
                    indexerSub.getPattern(),
                    turretSub.getMotif(),
                    indexerSub.isCenterControl()
                );
            }
            if (shooterSM.isShooting()) {
                shooterSM.update(telemetry);
            }

            // Sync pattern after automated shooting completes
            if (!shooterSM.isShooting() && "XXX".equals(shooterSM.getPattern())) {
                indexerSub.setPattern("XXX");
                indexerSub.setCenterControl(false);
            }

            // =============================================================
            // INTAKE TOGGLE (RIGHT TRIGGER)
            // =============================================================
            if ((gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5)
                && rightTriggerDebounce.milliseconds() > 500) {
                intakeToggle = !intakeToggle;
                rightTriggerDebounce.reset();
            }
            intakeSub.run(intakeToggle);

            // =============================================================
            // CENTER CONTROL (LEFT TRIGGER or auto when all balls loaded)
            // =============================================================
            pattern = indexerSub.getPattern(); // Re-read pattern after possible changes

            if ((PatternUtil.count(pattern, x1) == 0
                || ((gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5)
                    && leftTriggerDebounce.milliseconds() > 500))
                && !indexerSub.isCenterControl()) {

                // Pre-position indexer for shooting
                indexerSub.update(shooterSM.isShooting(), manualShooting,
                    turretSub.getMotif(), telemetry);
                leftTriggerDebounce.reset();
            }

            // =============================================================
            // INDEXER COLOR DETECTION (when not shooting)
            // =============================================================
            if (!intakeBool) {
                intakeDelay.reset();
            }
            if (intakeDelay.milliseconds() > 750) {
                intakeBool = false;
            }

            if (!intakeBool) {
                indexerSub.update(shooterSM.isShooting(), manualShooting,
                    turretSub.getMotif(), telemetry);
            }

            // =============================================================
            // TELEMETRY
            // =============================================================
            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Pattern", indexerSub.getPattern());
            telemetry.addData("Turret Tracking", turretTrackingEnabled ? "ON" : "OFF");
            telemetry.addData("Turret Position", hw.turret.getCurrentPosition());
            telemetry.addData("Robot X", follower.getPose().getX());
            telemetry.addData("Robot Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
            telemetry.addData("Shooter Velocity", hw.shooter.getVelocity());
            telemetry.update();
        }
    }
}
