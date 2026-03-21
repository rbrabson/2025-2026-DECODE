package org.firstinspires.ftc.teamcode.telemetry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * A Telemetry implementation that forwards all calls to multiple underlying Telemetry instances.
 * This allows you to send telemetry data to multiple destinations (e.g., driver station and dashboard)
 * simultaneously.
 */
public class MultiTelemetry implements TelemetryEx {
    private final Telemetry[] telemetry;
    private Level level;

    /**
     * Creates a MultiTelemetry instance that forwards to the given Telemetry instances with INFO level.
     *
     * @param telemetries the Telemetry instances to forward to
     */
    public MultiTelemetry(@NonNull Telemetry... telemetries) {
        this(Level.INFO, telemetries);
    }

    /**
     * Creates a MultiTelemetry instance that forwards to the given Telemetry instances.
     *
     * @param level the TelemetryEx.Level to use for all forwarded telemetry data
     * @param telemetries the Telemetry instances to forward to
     */
    public MultiTelemetry(@NonNull Level level, @NonNull Telemetry... telemetries) {
        this.level = Objects.requireNonNull(level);
        this.telemetry = Objects.requireNonNull(telemetries);
        for (Telemetry t : telemetries) {
            if (t instanceof TelemetryEx) {
                ((TelemetryEx) t).setLevel(level);
            }
        }
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
        Item item = null;
        if (this.level.compareTo(level) <= 0) {
            for (Telemetry t : telemetry) {
                Item single = t.addData(caption, format, args);
                if (single != null) {
                    item = single;
                }
            }
        }
        return item;
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
        Item item = null;
        if (this.level.compareTo(level) <= 0) {
            for (Telemetry t : telemetry) {
                Item single = t.addData(caption, value);
                if (single != null) {
                    item = single;
                }
            }
        }
        return item;
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
        Item item = null;
        if (this.level.compareTo(level) <= 0) {
            for (Telemetry t : telemetry) {
                Item single = t.addData(caption, valueProducer);
                if (single != null) {
                    item = single;
                }
            }
        }
        return item;
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
        Line line = null;
        if (this.level.compareTo(level) <= 0) {
            for (Telemetry t : telemetry) {
                Line single = t.addLine(lineCaption);
                if (single != null) {
                    line = single;
                }
            }
        }
        return line;
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
        for (Telemetry t : telemetry) {
            if (t instanceof TelemetryEx) {
                ((TelemetryEx) t).setLevel(level);
            }
        }
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        Item item = null;
        for (Telemetry t : telemetry) {
            Item single = t.addData(caption, format, args);
            if (single != null) {
                item = single;
            }
        }
        return item;
    }

    @Override
    public Item addData(String caption, Object value) {
        Item item = null;
        for (Telemetry t : telemetry) {
            Item single = t.addData(caption, value);
            if (single != null) {
                item = single;
            }
        }
        return item;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        Item item = null;
        for (Telemetry t : telemetry) {
            Item single = t.addData(caption, valueProducer);
            if (single != null) {
                item = single;
            }
        }
        return item;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        Item item = null;
        for (Telemetry t : telemetry) {
            Item single = t.addData(caption, format, valueProducer);
            if (single != null) {
                item = single;
            }
        }
        return item;
    }

    @Override
    public boolean removeItem(Item item) {
        boolean removed = false;
        for (Telemetry t : telemetry) {
            if (t.removeItem(item)) {
                removed = true;
            }
        }
        return removed;
    }

    @Override
    public void clear() {
        for (Telemetry t : telemetry) {
            t.clear();
        }
    }

    @Override
    public void clearAll() {
        for (Telemetry t : telemetry) {
            t.clearAll();
        }
    }

    @Override
    public Object addAction(Runnable action) {
        Object token = null;
        for (Telemetry t : telemetry) {
            Object single = t.addAction(action);
            if (single != null) {
                token = single;
            }
        }
        return token;
    }

    @Override
    public boolean removeAction(Object token) {
        boolean removed = false;
        for (Telemetry t : telemetry) {
            if (t.removeAction(token)) {
                removed = true;
            }
        }
        return removed;
    }

    @Override
    public void speak(String text) {
        for (Telemetry t : telemetry) {
            t.speak(text);
        }
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        for (Telemetry t : telemetry) {
            t.speak(text, languageCode, countryCode);
        }
    }

    @Override
    public boolean update() {
        boolean updated = false;
        for (Telemetry t : telemetry) {
            if (t.update()) {
                updated = true;
            }
        }
        return updated;
    }

    @Override
    public Line addLine() {
        Line line = null;
        for (Telemetry t : telemetry) {
            Line single = t.addLine();
            if (single != null) {
                line = single;
            }
        }
        return line;
    }

    @Override
    public Line addLine(String lineCaption) {
        Line line = null;
        for (Telemetry t : telemetry) {
            Line single = t.addLine(lineCaption);
            if (single != null) {
                line = single;
            }
        }
        return line;
    }

    @Override
    public boolean removeLine(Line line) {
        boolean removed = false;
        for (Telemetry t : telemetry) {
            if (t.removeLine(line)) {
                removed = true;
            }
        }
        return removed;
    }

    @Override
    public boolean isAutoClear() {
        boolean autoClear = false;
        for (Telemetry t : telemetry) {
            if (t.isAutoClear()) {
                autoClear = true;
                break;
            }
        }
        return autoClear;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        for (Telemetry t : telemetry) {
            t.setAutoClear(autoClear);
        }
    }

    @Override
    public int getMsTransmissionInterval() {
        int msTransmissionInterval = 0;
        for (Telemetry t : telemetry) {
            msTransmissionInterval = Math.max(msTransmissionInterval, t.getMsTransmissionInterval());
        }
        return msTransmissionInterval;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        for (Telemetry t : telemetry) {
            t.setMsTransmissionInterval(msTransmissionInterval);
        }
    }

    @Override
    public String getItemSeparator() {
        String separator = "";
        for (Telemetry t : telemetry) {
            String single = t.getItemSeparator();
            if (single != null && !single.isEmpty()) {
                separator = single;
                break;
            }
        }

        return separator;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        for (Telemetry t : telemetry) {
            t.setItemSeparator(itemSeparator);
        }
    }

    @Override
    public String getCaptionValueSeparator() {
        String separator = "";
        for (Telemetry t : telemetry) {
            String single = t.getCaptionValueSeparator();
            if (single != null && !single.isEmpty()) {
                separator = single;
                break;
            }
        }

        return separator;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        for (Telemetry t : telemetry) {
            t.setCaptionValueSeparator(captionValueSeparator);
        }
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        for (Telemetry t : telemetry) {
            t.setDisplayFormat(displayFormat);
        }
    }

    @Override
    public Log log() {
        Log log = null;
        for (Telemetry t : telemetry) {
            Log single = t.log();
            if (single != null) {
                log = single;
            }
        }
        return log;
    }
}
