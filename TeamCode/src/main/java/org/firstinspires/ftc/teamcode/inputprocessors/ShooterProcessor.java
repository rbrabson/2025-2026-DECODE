package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;

import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.Alliance;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.sensors.Limelight;

import java.util.Objects;

/**
 * ShooterProcessor handles the input processing for the shooter mechanism, including toggling automation,
 * adjusting flywheel speed, and providing feedback on turret alignment.
 */
public class ShooterProcessor implements UserInputProcessor {
    private static final double HOOD_INCREMENT = 0.05;
    private static final double ACCEPTABLE_TURRET_ERROR = 1.0;

    private final Shooter shooter;
    private final Limelight limelight;
    private final Localizer localizer;
    private final Alliance alliance;
    private final Telemetry telemetry;

    private boolean automateShooting = true;

    private boolean turretAligned = false;

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
        this.shooter = Objects.requireNonNull(shooter);
        this.limelight = Objects.requireNonNull(limelight);
        this.localizer = Objects.requireNonNull(localizer);
        this.alliance = Objects.requireNonNull(alliance);
        this.telemetry = Objects.requireNonNull(telemetry);
    }

    /**
     * Process the gamepad inputs to control the shooter mechanism.
     *
     * @param gamepad1 The current state of gamepad1.
     * @param gamepad2 The current state of gamepad2.
     */
    @Override public void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        // Toggle automated shooting
        if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
            automateShooting = !automateShooting;
        }

        // Shot compensation on gamepad1 dpad
        if (gamepad1.dpadLeftWasPressed()) {
            shooter.shotWasHigh();
        } else if (gamepad1.dpadRightWasPressed()) {
            shooter.shotWasLow();
        }

        // Hood adjustment on gamepad2 dpad
        if (gamepad2.dpadLeftWasPressed()) {
            shooter.decreaseHoodPosition(HOOD_INCREMENT);
        } else if (gamepad2.dpadRightWasPressed()) {
            shooter.increaseHoodPosition(HOOD_INCREMENT);
        }

        // RPM adjustment on gamepad2 dpad
        if (gamepad2.dpadUpWasPressed()) {
            shooter.increaseFlywheelRPM(50);
        } else if (gamepad2.dpadDownWasPressed()) {
            shooter.decreaseFlywheelRPM(50);
        }

        // Turret adjustment on gamepad2 x/b buttons
        if (gamepad2.xWasPressed()) {
            shooter.rotateTurretLeft(100);
        } else if (gamepad2.bWasPressed()) {
            shooter.rotateTurretRight(100);
        }

        double llError = limelight.getError();
        if (automateShooting) {
            boolean aligned = !Double.isNaN(llError) && Math.abs(llError) <= ACCEPTABLE_TURRET_ERROR;
            // Only rumble the once when the turret is aligned
            if (aligned && !turretAligned) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }
            turretAligned = aligned;
        } else {
            turretAligned = false;
        }

        telemetry.addData("[SHOOTER] Flywheel RPM", shooter.getFlywheelRPM());
        telemetry.addData("[SHOOTER] Hood Position", shooter.getHoodPosition());
        telemetry.addData("[SHOOTER] Turret Position}", shooter.getTurretCurrentPosition());
        telemetry.addData("[SHOOTER] Distance", shooter.getDistanceToTarget());
        telemetry.addData("[SHOOTER] Limelight Error", llError);
    }
}
