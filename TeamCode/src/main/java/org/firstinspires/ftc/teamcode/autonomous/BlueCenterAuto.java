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
 * Blue Center Autonomous OpMode.
 *
 * <p>This autonomous routine starts from the center position on the blue alliance side.
 * It mirrors the general strategy of RedCenterAuto but with poses reflected for the
 * blue side of the field. The robot drives to the shooting position, fires the
 * preloaded balls, then cycles through three floor pickups -- collecting each one,
 * pushing the first through the gate, returning to the shooting pose to score, and
 * finally parking at the end position.</p>
 *
 * <p>Note: Some intake paths use constant heading interpolation at 180 degrees
 * to keep the robot facing the wall while pushing balls, and reduced braking
 * strength for smoother ball collection.</p>
 *
 * <p>Replaces the legacy JWBC OpMode.</p>
 */
@Autonomous(name = "Blue Center")
public class BlueCenterAuto extends BaseAutonomous {

    // ---------------------------------------------------------------
    // Pose definitions
    // TODO: [Student] Compare these poses with RedCenterAuto. How are they
    //       mirrored for the blue side? Which axis is the mirror on?
    // ---------------------------------------------------------------

    /** Starting position on the field (center blue tile). */
    private final Pose startPose = new Pose(33.524861878453045, 135.69060773480663, Math.toRadians(90));

    /** Position the robot drives to in order to shoot at the blue goal. */
    private final Pose ShootPose = new Pose(47.84530386740332, 95.66850828729281, Math.toRadians(134));

    /** Approach poses for each of the three floor pickups. */
    private final Pose pickup1Pose = new Pose(44.57458563535911, 90.07734806629837, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(44.33149171270718, 65.674033149171294, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(44.563535911602216, 43.005524861878456, Math.toRadians(180));

    /** Intake poses -- the robot pushes forward here to collect the ball. */
    private final Pose intake1Pose = new Pose(25.96132596685084, 90.060773480663, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(25.386740331491723, 65.83425414364642, Math.toRadians(180));
    private final Pose intake3Pose = new Pose(25.41436464088398, 43.97237569060775, Math.toRadians(180));

    /** End / parking position after all cycles are complete. */
    private final Pose endpose = new Pose(32.03314917127072, 72.18232044198896, Math.toRadians(90));

    // Bezier curve control points (no heading -- used only as waypoints)
    // TODO: [Student] Why are these control point X-values lower than in
    //       RedCenterAuto? How does that relate to the blue side geometry?
    private final Pose Ctrl1 = new Pose(65.1767955801105, 82.73480662983425);
    private final Pose Ctrl2 = new Pose(64.58839779005525, 51.93646408839779);
    private final Pose Ctrl3 = new Pose(71.04972375690608, 30.70718232044198);
    private final Pose CtrlGate = new Pose(61.12435743454239, 78.60429978380974);

    /** The gate pose the robot drives through to deliver a ball to the scoring zone. */
    private final Pose Gate = new Pose(16.22651933701656, 78, Math.toRadians(90));

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
        return AllianceConfig.Alliance.BLUE;
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
    // TODO: [Student] Notice that the intake paths here use
    //       setConstantHeadingInterpolation(180 degrees) instead of
    //       setLinearHeadingInterpolation. Why would a constant heading
    //       be better for intaking on the blue side?
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

        // Push forward to intake the first ball (constant heading, reduced braking)
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.5)
                .build();

        // Drive through the gate to deliver the ball to the scoring zone
        // TODO: [Student] Why does HittingGate use constant heading at 90 degrees
        //       instead of linear interpolation here?
        HittingGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, CtrlGate, Gate))
                .setConstantHeadingInterpolation(Math.toRadians(90))
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

        // Push forward to intake the second ball (constant heading, reduced braking)
        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.5)
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

        // Push forward to intake the third ball (constant heading, reduced braking)
        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.5)
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
    // State machine (16 states -- same structure as RedCenterAuto)
    // TODO: [Student] This state machine is identical in structure to
    //       RedCenterAuto. Could the shared logic be extracted into a
    //       helper method? What would the trade-offs be?
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
