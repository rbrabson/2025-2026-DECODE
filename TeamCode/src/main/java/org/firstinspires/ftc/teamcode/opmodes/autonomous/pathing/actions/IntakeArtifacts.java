package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions;

import androidx.annotation.NonNull;

import com.rbrabson.behave.Node;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

/**
 * A Node that controls the Intake mechanism to pick up artifacts during autonomous.
 */
public class IntakeArtifacts implements Node {
    private final Intake intake;

    /**
     * Constructs an IntakeNode with the given Intake mechanism.
     * @param intake The Intake mechanism to control
     */
    public IntakeArtifacts(@NonNull Intake intake) {
        this.intake = intake;
    }

    /**
     * Executes the node's action to pick up artifacts using the Intake mechanism. This method
     * continues to try to pick up artifacts, always returning SUCCESS. This allows the node
     * to be used in a Parallel node where the robot is moving along a path while trying to pick up
     * artifacts at the same time.
     *
     * @return The status of the node after execution (SUCCESS)
     */
    @Override
    public Status tick() {
        intake.pickupArtifacts();
        return Status.SUCCESS;
    }

    /**
     * Resets the node's status to READY, allowing it to be ticked again from the beginning.
     *
     * @return The status of the node after resetting (READY)
     */
    @Override
    public Status reset() {
        return Status.READY;
    }

    /**
     * Returns the current status of the node.
     *
     * @return The current status of the node
     */
    @Override
    public Status status() {
        return Status.SUCCESS;
    }
}
