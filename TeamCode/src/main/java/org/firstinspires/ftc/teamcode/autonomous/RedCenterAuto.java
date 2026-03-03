package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hardware.IndexerConfig;
import org.firstinspires.ftc.teamcode.util.AllianceConfig;

/**
 * Red Center Autonomous OpMode.
 *
 * <p>This autonomous routine starts from the center position on the red alliance side.
 * The robot drives to the shooting position, fires the preloaded balls, then cycles
 * through three floor pickups -- collecting each one, pushing it through the gate,
 * returning to the shooting pose to score, and finally parking at the end position.</p>
 *
 * <p>Replaces the legacy JWRC OpMode.</p>
 */
@Autonomous(name = "Red Center")
public class RedCenterAuto extends BaseAutonomous {

    // ---------------------------------------------------------------
    // Pose definitions
    // TODO: [Student] Why is the start pose heading 90 degrees? What direction does the robot face?
    // ---------------------------------------------------------------

    /** Starting position on the field (center red tile). */
    private final Pose startPose = new Pose(110.69613259668509, 135.6906077348066, Math.toRadians(90));

    /** Position the robot drives to in order to shoot at the goal. */
    private final Pose ShootPose = new Pose(96.57458563535913, 95.8674033149171, Math.toRadians(44));

    /** Approach poses for each of the three floor pickups. */
    private final Pose pickup1Pose = new Pose(102.62983425414362, 87.10497237569064, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(100.75690607734808, 61.889502762430936, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(101.21546961325969, 36.607734806629836, Math.toRadians(0));

    /** Intake poses -- the robot pushes forward here to collect the ball. */
    private final Pose intake1Pose = new Pose(119.49723756906077, 87.09944751381215, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(120.10497237569061, 61.707182320441994, Math.toRadians(0));
    private final Pose intake3Pose = new Pose(120.45856353591161, 36.68508287292818, Math.toRadians(0));

    /** End / parking position after all cycles are complete. */
    private final Pose endpose = new Pose(112.08287292817678, 72.17679558011054, Math.toRadians(90));

    // Bezier curve control points (no heading -- used only as waypoints)
    // TODO: [Student] What is the purpose of these control points in a BezierCurve?
    private final Pose Ctrl1 = new Pose(78.71823204419888, 83.99999999999999);
    private final Pose Ctrl2 = new Pose(78.37292817679558, 59.17955801104973);
    private final Pose Ctrl3 = new Pose(76.40883977900553, 40.27624309392262);
    private final Pose CtrlGate = new Pose(82.87564256545761, 67.39570021619026);

    /** The gate pose the robot drives through to deliver a ball to the scoring area. */
    private final Pose Gate = new Pose(127.3922651933702, 79.98895027624306, Math.toRadians(90));

    // ---------------------------------------------------------------
    // Path chains
    // ---------------------------------------------------------------

    private PathChain scorePreload;
    private PathChain grabPickup1, intakePickup1, scorePickup1;
    private PathChain grabPickup2, intakePickup2, scorePickup2;
    private PathChain grabPickup3, intakePickup3, scorePickup3;
    private PathChain HittingGate;
    private PathChain ending;

    // ---------------------------------------------------------------
    // One-shot flags for triggering shots
    // ---------------------------------------------------------------

    private boolean flag1 = true;
    private boolean flag2 = true;
    private boolean flag3 = true;
    private boolean flag4 = true;

    // ---------------------------------------------------------------
    // BaseAutonomous abstract method implementations
    // ---------------------------------------------------------------

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected AllianceConfig.Alliance getAlliance() {
        return AllianceConfig.Alliance.RED;
    }

    @Override
    protected IndexerConfig getIndexerConfig() {
        return IndexerConfig.CONFIG_B;
    }

    @Override
    protected int[] getShootingDelays() {
        return new int[]{1000, 1175, 1350};
    }

    @Override
    protected String getInitialPattern() {
        return "GPP";
    }

    // ---------------------------------------------------------------
    // Path building
    // TODO: [Student] Trace each path on the field diagram. Why do some paths
    //       use BezierCurve while others use BezierLine?
    // ---------------------------------------------------------------

    @Override
    public void buildPaths(Follower follower) {
        // Drive from start to the shooting position with the preloaded balls
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose.getHeading())
                .build();

        // Curve from the shooting pose to the first pickup approach point
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Push forward to intake the first ball
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), intake1Pose.getHeading())
                .build();

        // Drive through the gate to deliver the ball to the scoring zone
        // TODO: [Student] Why does the robot need to pass through a gate here?
        HittingGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, CtrlGate, Gate))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), Gate.getHeading())
                .build();

        // Return from the gate to the shooting position to score pickup 1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Gate, ShootPose))
                .setLinearHeadingInterpolation(Gate.getHeading(), ShootPose.getHeading())
                .build();

        // Curve from the shooting pose to the second pickup approach point
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl2, pickup2Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Push forward to intake the second ball
        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), intake2Pose.getHeading())
                .build();

        // Drive directly from the second intake to the shooting pose to score
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, ShootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), ShootPose.getHeading())
                .build();

        // Curve from the shooting pose to the third pickup approach point
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl3, pickup3Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Push forward to intake the third ball
        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), intake3Pose.getHeading())
                .build();

        // Drive from the third intake to the shooting pose to score
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, ShootPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), ShootPose.getHeading())
                .build();

        // Drive from the shooting pose to the parking / end position
        ending = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, endpose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), endpose.getHeading())
                .build();
    }

    // ---------------------------------------------------------------
    // State machine
    // TODO: [Student] Draw a flowchart of these states. Which states wait for
    //       the robot to stop moving, and which wait for shooting to finish?
    // ---------------------------------------------------------------

    @Override
    public void autonomousPathUpdate() {
        switch (pathState) {

            // --- Preload cycle ---

            case 0:
                // Start driving to the shooting position with preloaded balls
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // Wait until the robot arrives, then trigger the preload shot
                if (!follower.isBusy()) {
                    if (flag1) {
                        triggerShoot();
                        flag1 = false;
                    }
                    setPathState(2);
                }
                break;

            case 2:
                // Once shooting is done, head to the first pickup
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup1);
                    setPathState(3);
                }
                break;

            // --- Pickup 1 cycle ---

            case 3:
                // Wait for arrival at pickup 1 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup1);
                    setPathState(4);
                }
                break;

            case 4:
                // After intaking, drive through the gate
                if (!follower.isBusy()) {
                    follower.followPath(HittingGate);
                    setPathState(5);
                }
                break;

            case 5:
                // After passing through the gate, return to the shooting pose
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(6);
                }
                break;

            case 6:
                // Arrived at shooting pose -- fire pickup 1
                if (!follower.isBusy()) {
                    if (flag2) {
                        triggerShoot();
                        flag2 = false;
                    }
                    setPathState(7);
                }
                break;

            case 7:
                // Once shooting is done, head to the second pickup
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup2);
                    setPathState(8);
                }
                break;

            // --- Pickup 2 cycle ---

            case 8:
                // Wait for arrival at pickup 2 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup2);
                    setPathState(9);
                }
                break;

            case 9:
                // After intaking, drive to the shooting pose to score
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(10);
                }
                break;

            case 10:
                // Arrived at shooting pose -- fire pickup 2
                if (!follower.isBusy()) {
                    if (flag3) {
                        triggerShoot();
                        flag3 = false;
                    }
                    setPathState(11);
                }
                break;

            case 11:
                // Once shooting is done, head to the third pickup
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup3);
                    setPathState(12);
                }
                break;

            // --- Pickup 3 cycle ---

            case 12:
                // Wait for arrival at pickup 3 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup3);
                    setPathState(13);
                }
                break;

            case 13:
                // After intaking, drive to the shooting pose to score
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(14);
                }
                break;

            case 14:
                // Arrived at shooting pose -- fire pickup 3
                if (!follower.isBusy()) {
                    if (flag4) {
                        triggerShoot();
                        flag4 = false;
                    }
                    setPathState(15);
                }
                break;

            // --- End / park ---

            case 15:
                // Once the final shot is done, drive to the parking position
                if (!shooter.isShooting()) {
                    follower.followPath(ending);
                    setPathState(-1);
                }
                break;
        }
    }
}
