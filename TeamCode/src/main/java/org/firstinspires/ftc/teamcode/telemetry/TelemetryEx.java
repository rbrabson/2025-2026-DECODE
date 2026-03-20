package org.firstinspires.ftc.teamcode.telemetry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface TelemetryEx extends Telemetry {
    /**
     * Log levels in increasing order of severity. Messages with a level less than or equal to the
     * current log level will be sent to telemetry.
     */
    public enum Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR
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
    public Item debugData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item infoData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item warningData(String caption, String format, Object... args);

    /**
     * Adds a telemetry data item with the specified caption and formatted value if the specified log
     * level is less than or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param format  The format string for the value, using String.format syntax.
     * @param args    The arguments to be formatted into the value string.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item errorData(String caption, String format, Object... args);

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
    public Item addData(TelemetryExImpl.Level level, String caption, String format, Object... args);

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item debugData(String caption, Object value) ;

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item infoData(String caption, Object value);

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item warningData(String caption, Object value) ;

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item errorData(String caption, Object value);

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param level   The log level for this telemetry data item.
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item addData(TelemetryExImpl.Level level, String caption, Object value);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item debugData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item infoData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item warningData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item errorData(String caption, Func<T> valueProducer);

    /** Adds a telemetry data item with the specified caption and value produced by the given function
     * if the specified log level is less than or equal to the current log level. Otherwise, does
     * nothing and returns null.
     *
     * @param level         The log level for this telemetry data item.
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item addData(TelemetryExImpl.Level level, String caption, Func<T> valueProducer);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line debugLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line infoLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line warningLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line errorLine(String lineCaption);

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param level       The log level for this telemetry line.
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line addLine(TelemetryExImpl.Level level, String lineCaption);

    /**
     * Gets the current log level. Messages with a level less than or equal to this level will be
     * sent to telemetry, while messages with a higher level will be ignored.
     *
     * @return The current log level.
     */
    public TelemetryExImpl.Level getLevel();

    /**
     * Sets the current log level. Messages with a level less than or equal to this level will be sent to
     * telemetry, while messages with a higher level will be ignored.
     *
     * @param level The log level to set. Must not be null.
     */
    public void setLevel(@NonNull TelemetryExImpl.Level level);
}
