package org.firstinspires.ftc.teamcode.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Wrapper class for the VoltageSensor hardware component.
 * Provides a method to get the current voltage of the robot.
 */
public class VoltageSensor {
    private final Iterable<com.qualcomm.robotcore.hardware.VoltageSensor> sensors;
    private final Telemetry telemetry;

    /**
     * Constructor for the VoltageSensor class.
     *
     * @param hardwareMap The hardware map to access the voltage sensor.
     * @param telemetry   The telemetry object for logging voltage readings.
     */
    public VoltageSensor(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        if (!map.voltageSensor.iterator().hasNext()) {
            throw new IllegalStateException("No voltage sensors found in HardwareMap");
        }
        this.sensors = map.voltageSensor;
        this.telemetry = Objects.requireNonNull(telemetry);
    }

    /**
     * Gets the current voltage of the robot.
     *
     * @return The current voltage reading from the voltage sensor.
     */
    public double getVoltage() {
        double minPositiveVoltage = Double.POSITIVE_INFINITY;

        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : sensors) {
            double v = sensor.getVoltage();
            if (v > 0.0 && v < minPositiveVoltage) {
                minPositiveVoltage = v;
            }
        }

        double result = Double.isInfinite(minPositiveVoltage) ? 0.0 : minPositiveVoltage;

        telemetry.addData("Battery Voltage", result);

        return result;
    }

}
