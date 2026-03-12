package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

/**
 * Utility class for mathematical operations commonly used in FTC robot programming.
 */
public class MathEx {

    /**
     * Returns the maximum absolute value from the provided values. This is used to
     * normalize motor powers so that no motor power exceeds 1.0, while maintaining
     * the proportionality of the powers.
     *
     * @param values the values to compare
     * @return the maximum absolute value among the provided values
     */
    public static double maxAbs(@NonNull double... values) {
        double max = 0.0;
        for (double v : values) {
            max = Math.max(max, Math.abs(v));
        }
        return max;
    }

    /**
     * Wraps an angle in radians to the range [-π, π]. This is useful for ensuring that the angle
     * between the target heading and the current heading is always represented as the shortest angle.
     *
     * @param radians the angle in radians to wrap
     * @return the wrapped angle in radians, constrained to the range [-π, π]
     */
    public static double angleWrap(double radians) {
        return Math.IEEEremainder(radians, 2 * Math.PI);
    }

    /**
     * Performs linear interpolation between two values a and b, using the specified weight. The
     * weight should be between 0 and 1, where 0 will return a, 1 will return b, and values in
     * between will return a blend of the two.
     *
     * @param a The first value to interpolate.
     * @param b The second value to interpolate.
     * @param weight The weight for interpolation, between 0 and 1. A weight of 0 will
     *               return a, a weight of 1 will return b.
     * @return The interpolated value between a and b based on the specified weight.
     */
    public static double lerp(double a, double b, double weight) {
        return a + weight * (b - a);
    }
}
