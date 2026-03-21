package org.firstinspires.ftc.teamcode.telemetry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An extension of the Telemetry interface that supports log levels for telemetry data. This allows
 * you to control which telemetry messages are sent based on their severity, making it easier to
 * filter out less important information during operation.
 */
public interface TelemetryEx extends Telemetry {
    /**
     * Wraps a Telemetry object in a TelemetryEx implementation with the default log level of INFO.
     * The returned TelemetryExImpl will send messages with level INFO, WARNING, and ERROR to the
     * given Telemetry object, but will ignore messages with level DEBUG.
     *
     * @param telemetry The Telemetry object to wrap. Must not be null.
     * @return A TelemetryEx implementation that wraps the given Telemetry object and uses the
     *         default log level of INFO.
     */
    @NonNull
    static TelemetryEx wrap(@NonNull Telemetry telemetry) {
        return TelemetryEx.wrap(Level.INFO, telemetry);
    }

    /**
     * Wraps a Telemetry object in a TelemetryEx implementation with the specified log level. The
     * returned TelemetryExImpl will send messages to the given Telemetry object if their level
     * is less than or equal to the specified log level.
     *
     * @param level      The log level for the returned TelemetryExImpl. Must not be null.
     * @param telemetry  The Telemetry object to wrap. Must not be null.
     * @return A TelemetryEx implementation that wraps the given Telemetry object and uses the
     *         specified log level.
     */
    @NonNull
    static TelemetryEx wrap(@NonNull Level level, @NonNull Telemetry telemetry) {
        return new TelemetryExImpl(level, telemetry);
    }

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item debugData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item infoData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item warningData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item errorData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param level   The log level for this telemetry data item.
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item addData(Level level, String caption, String format, Object... args);

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item debugData(String caption, Object value) ;

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item infoData(String caption, Object value);

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item warningData(String caption, Object value) ;

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item errorData(String caption, Object value);

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param level   The log level for this telemetry data item.
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    Item addData(Level level, String caption, Object value);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    <T> Item debugData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    <T> Item infoData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    <T> Item warningData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    <T> Item errorData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given function
     * if the specified log level is less than or equal to the current log level. Otherwise, does
     * nothing and returns null.
     *
     * @param level         The log level for this telemetry data item.
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    <T> Item addData(Level level, String caption, Func<T> valueProducer);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    Line debugLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    Line infoLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    Line warningLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    Line errorLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param level       The log level for this telemetry line.
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    Line addLine(Level level, String lineCaption);

    /**
     * Gets the current log level. Messages with a level less than or equal to this level will be
     * sent to telemetry, while messages with a higher level will be ignored.
     *
     * @return The current log level.
     */
    Level getLevel();

    /**
     * Sets the current log level. Messages with a level less than or equal to this level will be sent to
     * telemetry, while messages with a higher level will be ignored.
     *
     * @param level The log level to set. Must not be null.
     */
    void setLevel(@NonNull Level level);
}
