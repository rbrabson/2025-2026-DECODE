package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.rbrabson.behave.Node;
import com.rbrabson.behave.Status;


/**
 * A Node that follows a path using a Follower. This node will tick the follower and return its
 * status.
 */
public class FollowPath implements Node {
    private final Follower follower;
    private final PathChain pathChain;
    private Status status = Status.READY;

    /**
     * Creates a new FollowerNode with the given follower and path chain.
     * @param follower The follower to use for following the path.
     * @param pathChain The path chain to follow. The follower will be reset and set to follow
     *                  this path chain when the node is reset.
     */
    public FollowPath(@NonNull Follower follower, @NonNull PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
    }

    /**
     * Ticks the follower and returns its status. If the follower is not busy, this node will
     * return SUCCESS. If the follower is busy, this node will return RUNNING.
     *
     * @return The status of the follower. SUCCESS if the follower is not busy, RUNNING if the
     *         follower is busy.
     */
    @Override
    public Status tick() {
        if (status == Status.SUCCESS || status == Status.FAILURE) {
            return status;
        }

        if (status == Status.READY) {
            follower.followPath(pathChain);
            status = Status.RUNNING;
        }

        follower.update();

        if (!follower.isBusy()) {
            status = Status.SUCCESS;
        }

        return status;
    }


    /**
     * Resets the follower and sets it to follow the path chain. This node will return READY after
     * resetting.
     *
     * @return READY after resetting the follower and setting it to follow the path chain.
     */
    @Override
    public Status reset() {
        status = Status.READY;
        return status;
    }

    /**
     * Returns the current status of this node. This will be the status of the follower, which can be
     * READY, RUNNING, or SUCCESS.
     *
     * @return The current status of this node, which is the status of the follower.
     */
    @Override
    public Status status() {
        return status;
    }
}
