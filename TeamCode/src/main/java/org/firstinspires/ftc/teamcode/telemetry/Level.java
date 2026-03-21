package org.firstinspires.ftc.teamcode.telemetry;

import androidx.annotation.NonNull;

import java.util.*;

/**
 * Represents the severity level of a telemetry message. Higher values indicate higher severity.
 * Predefined levels include DEBUG (-10), INFO (0), WARNING (10), and ERROR (20). Custom levels
 * can be created with any integer value that is not already defined.
 */
public final class Level {
    private static final Map<Integer, Level> VALUES = new HashMap<>();

    public static final Level DEBUG = new Level(-10);
    public static final Level INFO = new Level(0);
    public static final Level WARNING = new Level(10);
    public static final Level ERROR = new Level(20);

    private final int value;

    /**
     * Creates a new Level with the specified integer value. If a level with the same value already
     * exists, the existing level will be returned instead of creating a new one.
     *
     * @param value the integer value representing the severity of the level
     */
    private Level(int value) {
        this.value = value;
        VALUES.put(value, this);
    }

    /**
     * Returns the Level instance corresponding to the given integer value. If no such level exists,
     * a new one will be created.
     *
     * @param value the integer value representing the severity of the level
     * @return the Level instance corresponding to the given integer value
     */
    public static Level of(int value) {
        return VALUES.computeIfAbsent(value, Level::new);
    }

    /**
     * Returns an unmodifiable collection of all defined Level instances, including both predefined
     * and custom levels.
     *
     * @return an unmodifiable collection of all defined Level instances
     */
    @NonNull
    public static Collection<Level> values() {
        return Collections.unmodifiableCollection(VALUES.values());
    }

    /**
     * Compares this level to another based on their integer values. Higher values indicate higher severity.
     *
     * @param other the other level to compare to
     * @return a negative integer, zero, or a positive integer as this level is less than, equal to, or greater than the specified level
     */
    public int compareTo(@NonNull Level other) {
        return Integer.compare(this.value, other.value);
    }

    /**
     * Returns a string representation of this level, which is simply its integer value. This can
     * be useful for debugging or logging purposes.
     *
     * @return a string representation of this level, which is its integer value
     */
    @NonNull
    @Override
    public String toString() {
        return Integer.toString(value);
    }
}
