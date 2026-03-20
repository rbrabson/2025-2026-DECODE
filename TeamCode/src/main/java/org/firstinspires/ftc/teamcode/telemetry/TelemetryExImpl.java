package org.firstinspires.ftc.teamcode.telemetry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * A wrapper around the Telemetry interface that adds support for log levels (DEBUG, INFO, WARNING, ERROR).
 * You can set the log level to control which messages are actually sent to telemetry.
 */
public class TelemetryExImpl implements TelemetryEx {
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

    private final Telemetry telemetry;
    private Level level;

    /**
     * Creates a TelemetryEx wrapper around the given Telemetry instance with the default log
     * level of INFO.
     *
     * @param telemetry The Telemetry instance to wrap. Must not be null.
     */
    public TelemetryExImpl(@NonNull Telemetry telemetry) {
        this(Level.INFO, telemetry);
    }

    /**
     * Creates a TelemetryEx wrapper around the given Telemetry instance with the specified log
     * level.
     *
     * @param level The initial log level to use.
     * @param telemetry The Telemetry instance to wrap. Must not be null.
     */
    public TelemetryExImpl(Level level, @NonNull Telemetry telemetry) {
        this.telemetry = Objects.requireNonNull(telemetry);
        this.level = Objects.requireNonNull(level);
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
    public Item debugData(String caption, String format, Object... args) {
        return addData(Level.DEBUG, caption, format, args);
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
    public Item infoData(String caption, String format, Object... args) {
        return addData(Level.INFO, caption, format, args);
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
    public Item warningData(String caption, String format, Object... args) {
        return addData(Level.WARNING, caption, format, args);
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
    public Item errorData(String caption, String format, Object... args) {
        return addData(Level.ERROR, caption, format, args);
    }

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
    public Item addData(Level level, String caption, String format, Object... args) {
        if (this.level.compareTo(level) <= 0) {
            return telemetry.addData(caption, format, args);
        }
        return null;
    }

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item debugData(String caption, Object value) {
        return addData(Level.DEBUG, caption, value);
    }

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item infoData(String caption, Object value) {
        return addData(Level.INFO, caption, value);
    }

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item warningData(String caption, Object value) {
        return addData(Level.WARNING, caption, value);
    }

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item errorData(String caption, Object value) {
        return addData(Level.ERROR, caption, value);
    }

    /** Adds a telemetry data item with the specified caption and value if the specified log level is less than
     * or equal to the current log level. Otherwise, does nothing and returns null.
     *
     * @param level   The log level for this telemetry data item.
     * @param caption The caption for the telemetry data item.
     * @param value   The value for the telemetry data item.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public Item addData(Level level, String caption, Object value) {
        if (this.level.compareTo(level) <= 0) {
            return telemetry.addData(caption, value);
        }
        return null;
    }

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item debugData(String caption, Func<T> valueProducer) {
        return addData(Level.DEBUG, caption, valueProducer);
    }

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item infoData(String caption, Func<T> valueProducer) {
        return addData(Level.INFO, caption, valueProducer);
    }

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item warningData(String caption, Func<T> valueProducer) {
        return addData(Level.WARNING, caption, valueProducer);
    }

    /** Adds a telemetry data item with the specified caption and value produced by the given
     * function if the specified log level is less than or equal to the current log level.
     * Otherwise, does nothing and returns null.
     *
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item errorData(String caption, Func<T> valueProducer) {
        return addData(Level.ERROR, caption, valueProducer);
    }

    /** Adds a telemetry data item with the specified caption and value produced by the given function
     * if the specified log level is less than or equal to the current log level. Otherwise, does
     * nothing and returns null.
     *
     * @param level         The log level for this telemetry data item.
     * @param caption       The caption for the telemetry data item.
     * @param valueProducer A function that produces the value for the telemetry data item when called.
     * @return The telemetry Item that was added, or null if the log level was too low to add the item.
     */
    public <T> Item addData(Level level, String caption, Func<T> valueProducer) {
        if (this.level.compareTo(level) <= 0) {
            return telemetry.addData(caption, valueProducer);
        }
        return null;
    }

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line debugLine(String lineCaption) {
        return addLine(Level.DEBUG, lineCaption);
    }

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line infoLine(String lineCaption) {
        return addLine(Level.INFO, lineCaption);
    }

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line warningLine(String lineCaption) {
        return addLine(Level.WARNING, lineCaption);
    }

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line errorLine(String lineCaption) {
        return addLine(Level.ERROR, lineCaption);
    }

    /** Adds a telemetry line with the specified caption if the specified log level is less than or
     *  equal to the current log level.  Otherwise, does nothing and returns null.
     *
     * @param level       The log level for this telemetry line.
     * @param lineCaption The caption for the telemetry line.
     * @return The telemetry Line that was added, or null if the log level was too low to add the line.
     */
    public Line addLine(Level level, String lineCaption) {
        if (this.level.compareTo(level) <= 0) {
            return telemetry.addLine(lineCaption);
        }
        return null;
    }

    /**
     * Gets the current log level. Messages with a level less than or equal to this level will be
     * sent to telemetry, while messages with a higher level will be ignored.
     *
     * @return The current log level.
     */
    public Level getLevel() {
        return level;
    }

    /**
     * Sets the current log level. Messages with a level less than or equal to this level will be sent to
     * telemetry, while messages with a higher level will be ignored.
     *
     * @param level The log level to set. Must not be null.
     */
    public void setLevel(@NonNull Level level) {
        this.level = Objects.requireNonNull(level);
    }

    // ---------- Telemetry interface methods ----------

    @Override
    public Item addData(String caption, String format, Object... args) {
        return telemetry.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        return telemetry.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return telemetry.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return telemetry.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        return telemetry.removeItem(item);
    }

    @Override
    public void clear() {
        telemetry.clear();
    }

    @Override
    public void clearAll() {
        telemetry.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        return telemetry.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return telemetry.removeAction(token);
    }

    @Override
    public void speak(String text) {
        telemetry.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        telemetry.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        return telemetry.update();
    }

    @Override
    public Line addLine() {
        return telemetry.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        return telemetry.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        return telemetry.removeLine(line);
    }

    @Override
    public boolean isAutoClear() {
        return telemetry.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        telemetry.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return telemetry.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        telemetry.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return telemetry.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        telemetry.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return telemetry.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        telemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        telemetry.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return telemetry.log();
    }
}
