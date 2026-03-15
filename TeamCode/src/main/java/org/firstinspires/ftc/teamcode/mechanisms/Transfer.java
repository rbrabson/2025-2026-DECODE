package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hinge;

import java.util.Objects;

/**
 * Mechanism class for transferring artifacts from the intake to the shooter.
 */
public class Transfer implements Mechanism {
    private final Hinge hinge;
    private final Telemetry telemetry;

    private TransferState transferState = TransferState.IDLE;

    /**
     * Constructor for the Transfer mechanism.
     *
     * @param hardwareMap The hardware map to initialize hardware devices.
     * @param telemetry   The telemetry object for logging.
     */
    public Transfer(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap);
        this.hinge = new Hinge(map, telemetry);
        this.telemetry = telemetry;
    }

    /**
     * Transfers an artifact from the intake to the shooter. This method handles the
     * entire process of moving the artifact, including controlling the hinge and
     * ensuring the artifact is in the correct position before shooting.
     *
     * @return The status of the transfer process, which can be RUNNING, SUCCESS, or FAILURE.
     */
    public Status transferArtifact() {
        switch (transferState) {
            case IDLE:
                hinge.raise();
                transferState = TransferState.RAISE_HINGE;
                telemetry.addData("Transfer", "Raise hinge");
                return Status.RUNNING;
            case RAISE_HINGE:
                if (!hinge.isRaised()) {
                    telemetry.addData("Transfer", "Hinge still raising");
                    return Status.RUNNING;
                }
                hinge.lower();
                transferState = TransferState.LOWER_HINGE;
                telemetry.addData("Transfer", "Lower hinge");
                return Status.RUNNING;
            case LOWER_HINGE:
                if (!hinge.isLowered()) {
                    telemetry.addData("Transfer", "Hinge still lowering");
                    return Status.RUNNING;
                }
                break;
        }

        transferState = TransferState.IDLE;
        telemetry.addData("Transfer", "Artifact transferred");
        return Status.SUCCESS;
    }

    @Override
    public void update() {
        // NO-OP
    }

    /**
     * Enum representing the states of the transfer process when moving an artifact from the
     * intake to the shooter.
     */
    private enum TransferState {
        /**
         * Hinge is not moving
         */
        IDLE,
        /**
         * Hinge is being raised
         */
        RAISE_HINGE,
        /**
         * Hinge is being lowered
         */
        LOWER_HINGE
    }
}
