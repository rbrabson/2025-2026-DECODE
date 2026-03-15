package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.hardware.VoltageSensor;

/**
 * Compensates motor powers based on the current voltage of the robot.
 * This helps maintain consistent performance even as the battery voltage drops.
 */
public class VoltageCompensator {
    private static final double NOMINAL = 12.5;

    private final VoltageSensor sensor;

    /**
     * Constructor for the VoltageCompensator class.
     * @param sensor The VoltageSensor instance to read the current voltage from.
     */
    public VoltageCompensator(@NonNull VoltageSensor sensor) {
        this.sensor = sensor;
    }

    /**
     * Compensates the given motor powers based on the current voltage reading.
     *
     * @param powers An array of motor powers to be compensated. The powers will be modified in place.
     * @return  The same array of motor powers after compensation has been applied.
     */
    @NonNull
    public double[] compensate(@NonNull double[] powers) {

        double voltage = sensor.getVoltage();
        double scale = NOMINAL / voltage;

        double max = 1;
        for (int i = 0; i < powers.length; i++) {
            powers[i] *= scale;
            max = Math.max(max, Math.abs(powers[i]));
        }
        // Normalize so that no power exceeds 1 after compensation
        if (max > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }

        return powers;
    }
}
