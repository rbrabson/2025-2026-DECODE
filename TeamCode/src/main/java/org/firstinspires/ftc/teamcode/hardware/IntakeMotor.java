package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Hardware class for the intake motor.
 */
public class IntakeMotor {
    private static final double INTAKE_POWER =1.0;

    private final DcMotorEx intake;
    private final Telemetry telemetry;

    /**
     * Constructor for the IntakeMotor class.
     *
     * @param hardwareMap The hardware map to initialize the intake motor.
     * @param telemetry The telemetry object for logging purposes.
     */
    public IntakeMotor(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.intake = map.get(DcMotorEx.class, "intake");
        this.telemetry = Objects.requireNonNull(telemetry);

        initializeIntakeMotor();

        this.telemetry.addLine("IntakeMotor initialized");
    }

    /**
     * Initializes the intake motor settings.
     */
    private void initializeIntakeMotor() {
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);
    }

    /**
     * Causes the intake motor to start spinning.
     */
    public void start() {
        intake.setPower(INTAKE_POWER);
    }

    /**
     * Causes the intake motor to stop spinning.
     */
    public void stop() {
        intake.setPower(0.0);
    }
}
