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
 * Blue alliance autonomous (far start) with CONFIG_B indexer and longer shooting delays.
 *
 * This is a variant of {@link BlueFarAuto} that uses CONFIG_B indexer configuration
 * and longer shooting delays (1000/1175/1350 ms instead of 500/675/850 ms). The
 * longer delays give the indexer more time to settle between shots, which can improve
 * accuracy at the cost of a slightly longer cycle time.
 *
 * <p><b>Key differences from BlueFarAuto:</b></p>
 * <ul>
 *   <li>Uses {@link IndexerConfig#CONFIG_B} instead of CONFIG_A</li>
 *   <li>Shooting delays are doubled (1000/1175/1350 ms)</li>
 *   <li>The HittingGate path is a straight line (no curve through CtrlGate)</li>
 *   <li>The final pickup 3 scoring and ending paths use ShootPose1 instead of ShootPose</li>
 *   <li>grabPickup3 starts from ShootPose1 instead of ShootPose</li>
 * </ul>
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
 * TODO: [Student] Compare this file with BlueFarAuto.java side by side. Only a
 *       handful of lines differ. The base class eliminates hundreds of lines of
 *       duplicated code. When you need to fix a bug in the shared logic (like
 *       turret tracking or intake control), you only fix it once in
 *       BaseAutonomous instead of in every autonomous file.
 *
 * TODO: [Student] Why might you choose CONFIG_B with longer delays over CONFIG_A?
 *       Consider tradeoffs between shooting accuracy and total cycle time. In a
 *       30-second autonomous period, every millisecond counts.
 */
@Autonomous(name = "Blue Far Finish")
public class BlueFarFinishAuto extends BaseAutonomous {

    // -----------------------------------------------------------------------
    // Poses (field coordinates and headings for each waypoint)
    // -----------------------------------------------------------------------

    // TODO: [Student] These poses are identical to BlueFarAuto. If you need to
    //       change a waypoint, remember to update it in both files (or consider
    //       extracting shared poses into a constants class).

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

    // TODO: [Student] The path chain names match BlueFarAuto, but some paths
    //       are built differently. See buildPaths() for the specific differences.

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
        return IndexerConfig.CONFIG_B;
    }

    @Override
    protected double[] getShootingDelays() {
        return new double[]{1000, 1175, 1350};
    }

    @Override
    protected String getInitialPattern() {
        return "GPP";
    }

    /**
     * Builds all path chains that the autonomous routine will follow.
     *
     * <p>Most paths are identical to {@link BlueFarAuto#buildPaths(Follower)},
     * with these exceptions:</p>
     * <ul>
     *   <li><b>HittingGate</b>: Uses a BezierLine (straight) instead of a
     *       BezierCurve through CtrlGate</li>
     *   <li><b>grabPickup3</b>: Starts from ShootPose1 instead of ShootPose</li>
     *   <li><b>scorePickup3</b>: Ends at ShootPose1 instead of ShootPose</li>
     *   <li><b>ending</b>: Starts from ShootPose1 instead of ShootPose</li>
     * </ul>
     *
     * TODO: [Student] Why might you use a straight line for the gate path in this
     *       variant? A BezierCurve provides a smoother arc but takes longer.
     *       A BezierLine is more direct. Consider which is better when time
     *       is tight due to the longer shooting delays.
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
                .addPath(new BezierLine(intake1Pose, Gate))
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
                .addPath(new BezierCurve(ShootPose1, Ctrl3, pickup3Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup3Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, ShootPose1))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), ShootPose1.getHeading())
                .build();

        ending = follower.pathBuilder()
                .addPath(new BezierLine(ShootPose1, endpose))
                .setLinearHeadingInterpolation(ShootPose1.getHeading(), endpose.getHeading())
                .build();
    }

    /**
     * State machine that sequences all autonomous actions.
     *
     * <p>This state machine is identical to {@link BlueFarAuto#autonomousPathUpdate()}.
     * The behavioral differences come from the different path chains built in
     * {@link #buildPaths(Follower)} and the longer shooting delays configured in
     * {@link #getShootingDelays()}.</p>
     *
     * TODO: [Student] The state machine code is duplicated between BlueFarAuto and
     *       BlueFarFinishAuto. Could this be extracted into the base class or a
     *       shared helper? What would the tradeoffs be? Sometimes a little
     *       duplication is acceptable if it keeps the code easy to modify
     *       independently for each variant.
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
