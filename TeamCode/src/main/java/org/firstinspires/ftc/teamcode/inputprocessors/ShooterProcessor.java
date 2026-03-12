package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;

import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.sensors.Limelight;

/**
 * ShooterProcessor handles the input processing for the shooter mechanism, including toggling automation,
 * adjusting flywheel speed, and providing feedback on turret alignment.
 */
public class ShooterProcessor implements InputProcessor {
    private static final double HOOD_INCREMENT = 0.05;
    private static final double ACCEPTABLE_TURRET_ERROR = 1.0;

    private final Shooter shooter;
    private final Limelight limelight;
    private final Localizer localizer;
    private final Alliance alliance;
    private final Telemetry telemetry;

    private boolean automateShooting = true;
    private boolean flywheelSpeedLow = true;

    private boolean leftBumperLatched = false;
    private boolean yLatched = false;
    private boolean gamepad1DpadLeftLatched = false;
    private boolean gamepad1DpadRightLatched = false;
    private boolean gamepad2DpadLeftLatched = false;
    private boolean gamepad2DpadRightLatched = false;
    private boolean turretAlignedLatched = false;

    /**
     * Constructor for the ShooterProcessor class.
     *
     * @param shooter   the Shooter mechanism to control
     * @param limelight the Limelight sensor for targeting
     * @param localizer the Localizer for robot position tracking
     * @param alliance  the Alliance to determine scoring positions
     * @param telemetry the Telemetry for debugging and feedback
     */
    public ShooterProcessor(@NonNull Shooter shooter, @NonNull Limelight limelight, @NonNull Localizer localizer, @NonNull Alliance alliance, @NonNull Telemetry telemetry) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.localizer = localizer;
        this.alliance = alliance;
        this.telemetry = telemetry;
    }

    /**
     * Process the gamepad inputs to control the shooter mechanism.
     *
     * @param gamepad1 The current state of gamepad1.
     * @param gamepad2 The current state of gamepad2.
     */
    @Override public void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        boolean leftBumperPressed = gamepad1.left_bumper || gamepad2.left_bumper;
        if (leftBumperPressed && !leftBumperLatched) {
            automateShooting = !automateShooting;
        }
        leftBumperLatched = leftBumperPressed;

        boolean yPressed = gamepad1.y || gamepad2.y;
        if (yPressed && !yLatched) {
            flywheelSpeedLow = !flywheelSpeedLow;
            if (flywheelSpeedLow) {
                shooter.setFlywheelVelocityToTeleopLow();
            } else {
                shooter.setFlywheelVelocityToTeleopHigh();
            }
        }
        yLatched = yPressed;

        // Shot compensation on gamepad1 dpad (edge-latched).
        boolean g1Left = gamepad1.dpad_left;
        if (g1Left && !gamepad1DpadLeftLatched) {
            shooter.shotWasHigh();
        }
        gamepad1DpadLeftLatched = g1Left;

        boolean g1Right = gamepad1.dpad_right;
        if (g1Right && !gamepad1DpadRightLatched) {
            shooter.shotWasLow();
        }
        gamepad1DpadRightLatched = g1Right;

        // Hood adjustment on gamepad2 dpad (edge-latched).
        boolean g2Left = gamepad2.dpad_left;
        if (g2Left && !gamepad2DpadLeftLatched) {
            shooter.decreaseHoodPosition(HOOD_INCREMENT);
        }
        gamepad2DpadLeftLatched = g2Left;

        boolean g2Right = gamepad2.dpad_right;
        if (g2Right && !gamepad2DpadRightLatched) {
            shooter.increaseHoodPosition(HOOD_INCREMENT);
        }
        gamepad2DpadRightLatched = g2Right;

        double llError = limelight.getError();
        if (automateShooting) {
            shooter.update(localizer, alliance);

            boolean aligned = !Double.isNaN(llError) && Math.abs(llError) <= ACCEPTABLE_TURRET_ERROR;
            if (aligned && !turretAlignedLatched) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }
            turretAlignedLatched = aligned;
        } else {
            turretAlignedLatched = false;
        }

        telemetry.addData("[SHOOTER] Auto", automateShooting);
        telemetry.addData("[SHOOTER] Flywheel Low", flywheelSpeedLow);
        telemetry.addData("[SHOOTER] Limelight Error", llError);
    }
}
