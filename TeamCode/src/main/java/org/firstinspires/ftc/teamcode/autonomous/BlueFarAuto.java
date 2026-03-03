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
 * Blue alliance autonomous that starts on the far side of the field.
 *
 * This OpMode extends {@link BaseAutonomous} and implements the full autonomous
 * routine for the blue alliance far starting position. It uses CONFIG_A indexer
 * timing and shorter shooting delays (500/675/850 ms).
 *
 * <p><b>Autonomous routine overview:</b></p>
 * <ol>
 *   <li>Score the preloaded balls by driving to the shooting position</li>
 *   <li>Pick up and score ball from pickup 4 (near the wall)</li>
 *   <li>Pick up ball from pickup 1 and push through the gate, then score</li>
 *   <li>Pick up and score ball from pickup 2</li>
 *   <li>Pick up and score ball from pickup 3</li>
 *   <li>Drive to the ending/parking position</li>
 * </ol>
 *
 * TODO: [Student] Study how the state machine sequences path following and
 *       shooting. Notice that the robot waits for the shooter to finish
 *       (via !shooter.isShooting()) before starting the next path. Why is
 *       this important? What would happen if the robot drove away while
 *       still shooting?
 *
 * TODO: [Student] Look at how the flag booleans (flag1-flag5) ensure that
 *       triggerShoot() is only called once per scoring cycle. Without these
 *       flags, the state machine would call triggerShoot() every loop
 *       iteration while in a shooting state.
 *
 * TODO: [Student] Compare this class with BlueFarFinishAuto.java. The only
 *       differences are the indexer config, shooting delays, and a few path
 *       variations. Both share the same base class infrastructure.
 */
@Autonomous(name = "Blue Far")
public class BlueFarAuto extends BaseAutonomous {

    // -----------------------------------------------------------------------
    // Poses (field coordinates and headings for each waypoint)
    // -----------------------------------------------------------------------

    // TODO: [Student] These poses define every position the robot visits.
    //       The coordinate system uses inches with (0,0) at the field corner.
    //       Headings are in radians. Try sketching these on a field diagram
    //       to visualize the autonomous path.

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose ShootPose = new Pose(47.84530386740332, 95.66850828729281, Math.toRadians(134));
    private final Pose ShootPose1 = new Pose(58.5, 13.723756906077352, Math.toRadians(118));
    private final Pose pickup1Pose = new Pose(44.57458563535911, 83.07734806629837, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(44.33149171270718, 58.83425414364642, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(44.563535911602216, 35.005524861878456, Math.toRadians(180));
    private final Pose pickup4Pose = new Pose(17.58011049723757, 10, Math.toRadians(180));
    private final Pose intake1Pose = new Pose(25.96132596685084, 83.060773480663, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(25.386740331491723, 58.83425414364642, Math.toRadians(180));
    private final Pose intake3Pose = new Pose(25.41436464088398, 35.97237569060775, Math.toRadians(180));
    private final Pose intake4Pose = new Pose(12.596685082872927, 10, Math.toRadians(180));
    private final Pose endpose = new Pose(32.03314917127072, 72.18232044198896, Math.toRadians(90));
    private final Pose Ctrl1 = new Pose(55.969613259668506, 48.85635359116024);
    private final Pose Ctrl2 = new Pose(64.58839779005525, 51.93646408839779);
    private final Pose Ctrl3 = new Pose(71.04972375690608, 30.70718232044198);
    private final Pose Ctrl4 = new Pose(32.07734806629835, 10.207182320441992);
    private final Pose CtrlGate = new Pose(55.12435743454239, 78.60429978380974);
    private final Pose Gate = new Pose(16.22651933701656, 78, Math.toRadians(90));

    // -----------------------------------------------------------------------
    // Path chains (built during init)
    // -----------------------------------------------------------------------

    // TODO: [Student] Each PathChain represents a segment of the autonomous route.
    //       "grab" paths drive to a pickup position, "intake" paths sweep the ball
    //       in, and "score" paths drive to the shooting position.

    private PathChain scorePreload, grabPickup4, intakePickup4, scorePickup4,
            grabPickup1, intakePickup1, HittingGate, scorePickup1,
            grabPickup2, intakePickup2, scorePickup2,
            grabPickup3, intakePickup3, scorePickup3, ending;

    // -----------------------------------------------------------------------
    // Shooting flags (ensure triggerShoot is called exactly once per cycle)
    // -----------------------------------------------------------------------

    private boolean flag1 = true;
    private boolean flag2 = true;
    private boolean flag3 = true;
    private boolean flag4 = true;
    private boolean flag5 = true;

    // -----------------------------------------------------------------------
    // BaseAutonomous abstract method implementations
    // -----------------------------------------------------------------------

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected AllianceConfig getAlliance() {
        return AllianceConfig.BLUE;
    }

    @Override
    protected IndexerConfig getIndexerConfig() {
        return IndexerConfig.CONFIG_A;
    }

    @Override
    protected double[] getShootingDelays() {
        return new double[]{500, 675, 850};
    }

    @Override
    protected String getInitialPattern() {
        return "GPP";
    }

    /**
     * Builds all path chains that the autonomous routine will follow.
     *
     * TODO: [Student] Each path uses either a BezierLine (straight line between
     *       two points) or a BezierCurve (curved path through control points).
     *       The heading interpolation controls which direction the robot faces
     *       while traveling along the path. Linear interpolation smoothly
     *       rotates between start and end headings; constant keeps a fixed heading.
     *
     * @param follower  the PedroPathing follower to build paths with
     */
    @Override
    protected void buildPaths(Follower follower) {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, pickup4Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup4Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        intakePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, intake4Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), intake4Pose.getHeading())
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, ShootPose1))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), ShootPose1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        HittingGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, CtrlGate, Gate))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), Gate.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Gate, ShootPose))
                .setLinearHeadingInterpolation(Gate.getHeading(), ShootPose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl2, pickup2Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup2Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, ShootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), ShootPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl3, pickup3Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup3Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, ShootPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), ShootPose.getHeading())
                .build();

        ending = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose, endpose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), endpose.getHeading())
                .build();
    }

    /**
     * State machine that sequences all autonomous actions.
     *
     * <p>The state machine alternates between three kinds of states:</p>
     * <ul>
     *   <li><b>Path-start states</b> (0, 2, 6, 11, 15): Begin following a path,
     *       then immediately advance to the next state.</li>
     *   <li><b>Wait-and-shoot states</b> (1, 5, 10, 14, 18): Wait for the robot
     *       to arrive at the scoring position, trigger shooting once, then advance.</li>
     *   <li><b>Post-shoot states</b> (2, 6, 11, 15, 19): Wait for shooting to
     *       finish, then start the next path segment.</li>
     * </ul>
     *
     * TODO: [Student] Trace through this state machine on paper. For each state,
     *       write down: (1) what condition must be true to proceed, (2) what action
     *       is taken, and (3) what state comes next. This is a common pattern in
     *       FTC autonomous programming.
     */
    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            // --- Score preloaded balls ---
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (flag1) {
                        triggerShoot();
                        flag1 = false;
                    }
                    setPathState(2);
                }
                break;

            // --- Pickup 4 cycle (near wall) ---
            case 2:
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup4);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup4);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup4);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (flag2) {
                        triggerShoot();
                        flag2 = false;
                    }
                    setPathState(6);
                }
                break;

            // --- Pickup 1 cycle (through gate) ---
            case 6:
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup1);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(HittingGate);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    if (flag3) {
                        triggerShoot();
                        flag3 = false;
                    }
                    setPathState(11);
                }
                break;

            // --- Pickup 2 cycle ---
            case 11:
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup2);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup2);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    if (flag4) {
                        triggerShoot();
                        flag4 = false;
                    }
                    setPathState(15);
                }
                break;

            // --- Pickup 3 cycle ---
            case 15:
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup3);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup3);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    if (flag5) {
                        triggerShoot();
                        flag5 = false;
                    }
                    setPathState(19);
                }
                break;

            // --- End / park ---
            case 19:
                if (!shooter.isShooting()) {
                    follower.followPath(ending);
                    setPathState(-1);
                }
                break;
        }
    }
}
