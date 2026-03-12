package org.firstinspires.ftc.teamcode.behave;

import androidx.annotation.NonNull;

import com.rbrabson.behave.Node;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The TelemetryNode class is a decorator node that wraps around another child node and logs its
 * status to telemetry. This allows you to easily monitor the execution of specific nodes in your
 * behavior tree by sending their status updates to the driver station.
 */
public class TelemetryNode implements Node {
    private final Node child;
    private final String message;
    private final Telemetry telemetry;

    private Status status = Status.READY;

    /**
     * Constructor takes a child node to decorate and the telemetry object to use for sending data
     * to the driver station.
     *
     * @param child     The child node to decorate.
     * @param telemetry The telemetry object to use for logging.
     */
    public TelemetryNode(@NonNull Node child, @NonNull Telemetry telemetry) {
        this(child, telemetry, "TelemetryNode status");
    }

    /**
     * Constructor takes a child node to decorate, the telemetry object to use for sending data
     * to the driver station, and an optional message to include.
     *
     * @param child     The child node to decorate.
     * @param telemetry The telemetry object to use for logging.
     * @param message   The message to log.
     */
    public TelemetryNode(@NonNull Node child, @NonNull Telemetry telemetry, @NonNull String message) {
        this.child = child;
        this.message = message;
        this.telemetry = telemetry;
    }

    /**
     * Executes the child node and logs its status to telemetry. If the child node is null or
     * returns a null status, this node will return FAILURE and log an appropriate message.
     */
    @Override
    public Status tick() {
        if (child == null) {
            status = Status.FAILURE;
            telemetry.addData("TelemetryNode has no child", status);
            return status;
        }
        Status childStatus = child.tick();
        if (childStatus == null) {
            status = Status.FAILURE;
            telemetry.addData("TelemetryNode child returned null status", status);
            return status;
        }
        status = childStatus;
        telemetry.addData(message, status);
        return status;
    }

    /**
     * Resets the status of this node and its child node (if it exists) to READY. This allows the
     * node to be re-executed from the beginning of its behavior tree branch.
     *
     * @return the status of this node after resetting, which will be READY unless the child node's
     *         reset method returns a different status.
     */
    @Override
    public Status reset() {
        status = Status.READY;
        if (child != null) {
            status = child.reset();
        }
        return status;
    }

    /**
     * Returns the current status of this node, which is determined by the last execution of the
     * tick method.
     *
     * @return the current status of this node, which can be READY, RUNNING, SUCCESS, or FAILURE
     *         depending on the last execution of the tick method
     */
    @Override
    public Status status() {
        return status;
    }

    /**
     * Provides a string representation of the node, including its current status.
     *
     * @return A string representation of this node.
     */
    @NonNull
    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("TelemetryNode (").append(status).append(")");
        if (child != null) {
            String[] lines = child.toString().split("\n");
            for (String line : lines) {
                builder.append("\n  ").append(line);
            }
        }
        return builder.toString();
    }
}
