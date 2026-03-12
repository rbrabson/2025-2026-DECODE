package org.firstinspires.ftc.teamcode.utils;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

/**
 * A simple 2D vector class for representing positions and directions in a 2D plane.
 */
public class Vector2 {
    public final double x;
    public final double y;

    /**
     * Constructor for the Vector2 class.
     * @param x The x component of the vector.
     * @param y The y component of the vector.
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Calculates the magnitude (length) of the vector.
     *
     * @return The magnitude of the vector.
     */
    public double magnitude() {
        return Math.hypot(x, y);
    }

    /**
     * Returns a new Vector2 that is the normalized (unit length) version of this vector.
     * If the magnitude is zero, it returns a zero vector to avoid division by zero.
     *
     * @return A new Vector2 that is the normalized version of this vector.
     */
    public Vector2 normalized() {
        double mag = magnitude();
        if (mag == 0.0) return new Vector2(0, 0);
        return new Vector2(x / mag, y / mag);
    }

    /**
     * Returns a new Vector2 that is the sum of this vector and the given vector.
     *
     * @param other The vector to add.
     * @return A new Vector2 representing the sum.
     */
    public Vector2 add(@NonNull Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    /**
     * Returns a new Vector2 that is the difference of this vector and the given vector.
     *
     * @param other The vector to subtract.
     * @return A new Vector2 representing the difference.
     */
    public Vector2 subtract(@NonNull Vector2 other) {
        return new Vector2(x - other.x, y - other.y);
    }

    /**
     * Returns a new Vector2 that is this vector scaled by the given scalar.
     *
     * @param scalar The scalar value to multiply by.
     * @return A new Vector2 representing the scaled vector.
     */
    public Vector2 scale(double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }

    /**
     * Calculates the dot product of this vector and the given vector.
     *
     * @param other The vector to dot with.
     * @return The scalar dot product.
     */
    public double dot(@NonNull Vector2 other) {
        return x * other.x + y * other.y;
    }

    /**
     * Rotates the vector by a given angle in radians and returns a new Vector2 representing the
     * rotated vector.
     *
     * @param angleRadians The angle to rotate the vector, in radians.
     * @return A new Vector2 that is the result of rotating this vector by the specified angle.
     */
    public Vector2 rotate(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);

        return new Vector2(
                x * cos - y * sin,
                x * sin + y * cos
        );
    }

    /**
     * Returns a string representation of the vector in the format "(x, y)" with three decimal places.
     *
     * @return A string representation of the vector.
     */
    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }
}
