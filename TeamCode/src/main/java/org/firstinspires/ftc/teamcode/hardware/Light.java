package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Class representing the light mechanism of the robot. The light is controlled
 * by a DC motor, which can be turned on or off.
 */
public class Light {
    private final DcMotorEx light;

    /**
     * Constructor for the Light class. Initializes the light motor and enables it.
     *
     * @param hardwareMap The hardware map to access the light motor.
     * @param telemetry   The telemetry object to log initialization status.
     */
    public Light(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(telemetry);
        light = map.get(DcMotorEx.class, "l");
        enable();

        telemetry.addLine("Light initialized");
    }

    /**
     * Enables the light by setting the motor power to 1.
     */
    public void enable() {
        light.setPower(1);
    }

    /**
     * Disables the light by setting the motor power to 0.
     */
    public void disable() {
        light.setPower(0);
    }
}
