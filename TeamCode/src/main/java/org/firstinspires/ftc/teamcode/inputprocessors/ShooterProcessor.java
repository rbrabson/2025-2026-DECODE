package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

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
public class ShooterProcessor implements UserInputProcessor {
    private static final double HOOD_INCREMENT = 0.05;
    private static final double ACCEPTABLE_TURRET_ERROR = 1.0;

    private final Shooter shooter;
    private final Limelight limelight;
    private final Localizer localizer;
    private final Alliance alliance;
    @Nullable private final Telemetry telemetry;

    private boolean automateShooting = true;
    private boolean flywheelSpeedLow = true;

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
    public ShooterProcessor(@NonNull Shooter shooter, @NonNull Limelight limelight, @NonNull Localizer localizer, @NonNull Alliance alliance, @Nullable Telemetry telemetry) {
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
        // Toggle automated shooting
        if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
            automateShooting = !automateShooting;
        }

        // Toggle flywheel speed
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            flywheelSpeedLow = !flywheelSpeedLow;
            if (flywheelSpeedLow) {
                shooter.setFlywheelRPMToLow();
            } else {
                shooter.setFlywheelRPMToHigh();
            }
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

        double llError = limelight.getError();
        if (automateShooting) {
            shooter.update(localizer, alliance);
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

        if (telemetry != null) {
            telemetry.addData("[SHOOTER] Auto", automateShooting);
            telemetry.addData("[SHOOTER] Flywheel Low", flywheelSpeedLow);
            telemetry.addData("[SHOOTER] Turret Aligned", turretAligned);
            telemetry.addData("[SHOOTER] Limelight Error", llError);
        }
    }
}
