package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.rbrabson.behave.Node;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

/**
 * This node is responsible for controlling the shooter mechanism during autonomous. It will
 * attempt to transfer an artifact from the intake to the shooter, and will return SUCCESS if the
 * transfer is successful, RUNNING if the transfer is still in progress, and FAILURE if the
 * transfer fails.
 */
public class ShootArtifacts implements Node {
    private final Follower follower;
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;

    private Status status = Status.READY;
    private boolean transferInProgress = false;

    /**
     * Constructor for ShootArtifacts. It takes in the shooter and intake mechanisms as parameters.
     *
     * @param follower The follower to get the robot's current pose for aiming the shooter.
     * @param shooter The shooter mechanism to control (not used in this node, but can be used for
     *                future extensions)
     * @param intake The intake mechanism to control, which is responsible for transferring artifacts
     *               to the shooter.
     * @param transfer The transfer mechanism to control, which is responsible for moving artifacts
     *                from the intake to the shooter.
     */
    public ShootArtifacts(@NonNull Follower follower, @NonNull Shooter shooter, @NonNull Intake intake, @NonNull Transfer transfer) {
        this.follower = follower;
        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
    }

    /**
     * This method is called to execute the node's behavior. It checks if the intake is empty,
     * and if so, it sets the status to SUCCESS. If the intake is not empty, it calls the
     * transferArtifactToShooter method on the intake to attempt to transfer an artifact to the
     * shooter, and sets the status to RUNNING while the transfer is in progress.
     *
     * @return SUCCESS if there are no more artifacts to shoot, RUNNING otherwise.
     */
    @Override
    public Status tick() {
        // Once there are no more artifacts to shoot, we're done.
        if (intake.isEmpty()) {
            status = Status.SUCCESS;
            return status;
        }

        // Continue trying to rotate the intake to a shooting position
        Status intakeStatus = intake.rotateToShootingPosition();
        if (intakeStatus == Status.FAILURE) {
            status = Status.SUCCESS;
            return status;
        }
        if (intakeStatus == Status.RUNNING) {
            status = Status.RUNNING;
            return status;
        }

        // Always update the turret position, even if the transfer is in progress, to ensure the
        // turret stays on target.
        shooter.setTurretTargetPosition(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

        // If the artifact transfer is in progress, finish it. Otherwise, initiate the transfer if
        // both the turret is in target position, and the flywheel is at the target velocity.
        if (transferInProgress || shooter.isReadyToShoot()) {
            Status transferStatus = transfer.transferArtifact();
            if (transferStatus == Status.SUCCESS) {
                intake.artifactTransferred();
            }
            transferInProgress = transferStatus == Status.RUNNING;
        }

        // Always keep running until there are no more artifacts to shoot
        status = Status.RUNNING;
        return status;
    }

    /**
     * This method is called to reset the node to its initial state. It sets the status back to READY.
     * @return The status after resetting, which is READY.
     */
    @Override
    public Status reset() {
        status = Status.READY;
        transferInProgress = false;
        return status;
    }

    /**
     * This method returns the current status of the node. It can be used to check the status
     * without ticking the node.
     *
     * @return The current status of the node, which can be READY, RUNNING, SUCCESS, or FAILURE
     *         depending on the state of the transfer.
     */
    @Override
    public Status status() {
        return status;
    }
}
