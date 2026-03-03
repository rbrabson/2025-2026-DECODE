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
 * Red Far Autonomous OpMode.
 *
 * <p>This autonomous routine starts from the far position on the red alliance side.
 * Unlike the center auto, this one begins near the wall and must first score its
 * preload from a nearby shooting pose (ShootPose1), then collect a fourth pickup
 * near the starting area before transitioning to the standard three-pickup cycle
 * with gate passes. The robot finishes by parking at the end position.</p>
 *
 * <p>Replaces the legacy JWRF OpMode.</p>
 */
@Autonomous(name = "Red Far")
public class RedFarAuto extends BaseAutonomous {

    // ---------------------------------------------------------------
    // Pose definitions
    // TODO: [Student] Compare these poses with RedCenterAuto. Why is the start
    //       pose so different? How does that affect the preload strategy?
    // ---------------------------------------------------------------

    /** Starting position on the field (far red tile, near the wall). */
    private final Pose startPose = new Pose(88, 7, Math.toRadians(90));

    /** Primary shooting position used after cycling the upper three pickups. */
    private final Pose ShootPose = new Pose(96.57458563535913, 95.8674033149171, Math.toRadians(52));

    /** Secondary shooting position near the start -- used for preload and pickup 4. */
    private final Pose ShootPose1 = new Pose(84.13812154696132, 17.734806629834296, Math.toRadians(69));

    /** Approach poses for the three upper floor pickups. */
    private final Pose pickup1Pose = new Pose(102.62983425414362, 83.10497237569064, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(100.75690607734808, 58.889502762430936, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(101.21546961325969, 34.607734806629836, Math.toRadians(0));

    /** Approach pose for the fourth pickup near the starting wall. */
    private final Pose pickup4Pose = new Pose(125.70165745856353, 8.552486187845302, Math.toRadians(0));

    /** Intake poses -- the robot pushes forward here to collect each ball. */
    private final Pose intake1Pose = new Pose(119.49723756906077, 83.09944751381215, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(120.10497237569061, 58.707182320441994, Math.toRadians(0));
    private final Pose intake3Pose = new Pose(120.45856353591161, 34.68508287292818, Math.toRadians(0));
    private final Pose intake4Pose = new Pose(135.82872928176798, 8.685082872928163, Math.toRadians(0));

    /** End / parking position after all cycles are complete. */
    private final Pose endpose = new Pose(112.08287292817678, 72.17679558011054, Math.toRadians(90));

    // Bezier curve control points (no heading -- used only as waypoints)
    // TODO: [Student] Why does Ctrl1 differ from the one in RedCenterAuto?
    private final Pose Ctrl1 = new Pose(87.07734806629834, 45.80662983425415);
    private final Pose Ctrl2 = new Pose(78.37292817679558, 59.17955801104973);
    private final Pose Ctrl3 = new Pose(76.40883977900553, 40.27624309392262);
    private final Pose Ctrl4 = new Pose(100.0524861878453, 7.392265193370193);
    private final Pose CtrlGate = new Pose(82.87564256545761, 65.39570021619026);

    /** The gate pose the robot drives through to deliver a ball to the scoring zone. */
    private final Pose Gate = new Pose(129.3922651933702, 73.98895027624306, Math.toRadians(90));

    // ---------------------------------------------------------------
    // Path chains
    // ---------------------------------------------------------------

    private PathChain scorePreload;
    private PathChain grabPickup4, intakePickup4, scorePickup4;
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
    private boolean flag5 = true;

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
    // TODO: [Student] Notice that this auto has a "pickup 4" cycle that the
    //       center auto does not. Why is that extra cycle needed here?
    // ---------------------------------------------------------------

    @Override
    public void buildPaths(Follower follower) {
        // Drive from start to the nearby shooting position for the preload
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())
                .build();

        // Drive from the nearby shoot pose to the fourth pickup (near the wall)
        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, pickup4Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup4Pose.getHeading())
                .setBrakingStrength(1)
                .build();

        // Push forward to intake the fourth ball
        intakePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, intake4Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), intake4Pose.getHeading())
                .build();

        // Return from the fourth intake to the nearby shooting position
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, ShootPose1))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), ShootPose1.getHeading())
                .build();

        // Curve from the main shooting pose to the first pickup approach point
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStrength(1)
                .build();

        // Push forward to intake the first ball
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .build();

        // Drive through the gate to deliver the ball to the scoring zone
        // TODO: [Student] Why does only the first pickup use the gate path?
        HittingGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, CtrlGate, Gate))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), Gate.getHeading())
                .build();

        // Return from the gate to the main shooting position
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Gate, ShootPose))
                .setLinearHeadingInterpolation(Gate.getHeading(), ShootPose.getHeading())
                .build();

        // Curve from the shooting pose to the second pickup approach point
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl2, pickup2Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup2Pose.getHeading())
                .setBrakingStrength(1)
                .build();

        // Push forward to intake the second ball
        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .build();

        // Drive directly from the second intake to the shooting pose
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, ShootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), ShootPose.getHeading())
                .build();

        // Curve from the shooting pose to the third pickup approach point
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl3, pickup3Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup3Pose.getHeading())
                .setBrakingStrength(1)
                .build();

        // Push forward to intake the third ball
        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .build();

        // Drive from the third intake to the nearby shooting pose (ShootPose1)
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, ShootPose1))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), ShootPose1.getHeading())
                .build();

        // Drive from the nearby shooting pose to the parking / end position
        ending = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, endpose))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endpose.getHeading())
                .build();
    }

    // ---------------------------------------------------------------
    // State machine (20 states)
    // TODO: [Student] This auto has 20 states while RedCenterAuto has 16.
    //       Identify the extra states and explain why they exist.
    // ---------------------------------------------------------------

    @Override
    public void autonomousPathUpdate() {
        switch (pathState) {

            // --- Preload cycle ---

            case 0:
                // Start driving to the nearby shooting position with preloaded balls
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
                // Once shooting is done, head to the fourth pickup (near the wall)
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup4);
                    setPathState(3);
                }
                break;

            // --- Pickup 4 cycle (near the starting wall) ---

            case 3:
                // Wait for arrival at pickup 4 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup4);
                    setPathState(4);
                }
                break;

            case 4:
                // After intaking, drive back to the nearby shooting pose
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup4);
                    setPathState(5);
                }
                break;

            case 5:
                // Arrived at nearby shooting pose -- fire pickup 4
                if (!follower.isBusy()) {
                    if (flag2) {
                        triggerShoot();
                        flag2 = false;
                    }
                    setPathState(6);
                }
                break;

            case 6:
                // Once shooting is done, head to the first upper pickup
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup1);
                    setPathState(7);
                }
                break;

            // --- Pickup 1 cycle ---

            case 7:
                // Wait for arrival at pickup 1 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup1);
                    setPathState(8);
                }
                break;

            case 8:
                // After intaking, drive through the gate
                if (!follower.isBusy()) {
                    follower.followPath(HittingGate);
                    setPathState(9);
                }
                break;

            case 9:
                // After passing through the gate, return to the main shooting pose
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(10);
                }
                break;

            case 10:
                // Arrived at shooting pose -- fire pickup 1
                if (!follower.isBusy()) {
                    if (flag3) {
                        triggerShoot();
                        flag3 = false;
                    }
                    setPathState(11);
                }
                break;

            case 11:
                // Once shooting is done, head to the second pickup
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup2);
                    setPathState(12);
                }
                break;

            // --- Pickup 2 cycle ---

            case 12:
                // Wait for arrival at pickup 2 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup2);
                    setPathState(13);
                }
                break;

            case 13:
                // After intaking, drive to the main shooting pose to score
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(14);
                }
                break;

            case 14:
                // Arrived at shooting pose -- fire pickup 2
                if (!follower.isBusy()) {
                    if (flag4) {
                        triggerShoot();
                        flag4 = false;
                    }
                    setPathState(15);
                }
                break;

            case 15:
                // Once shooting is done, head to the third pickup
                if (!shooter.isShooting()) {
                    follower.followPath(grabPickup3);
                    setPathState(16);
                }
                break;

            // --- Pickup 3 cycle ---

            case 16:
                // Wait for arrival at pickup 3 approach, then push forward to intake
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup3);
                    setPathState(17);
                }
                break;

            case 17:
                // After intaking, drive to the nearby shooting pose to score
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(18);
                }
                break;

            case 18:
                // Arrived at nearby shooting pose -- fire pickup 3
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
                // Once the final shot is done, drive to the parking position
                if (!shooter.isShooting()) {
                    follower.followPath(ending);
                    setPathState(-1);
                }
                break;
        }
    }
}
