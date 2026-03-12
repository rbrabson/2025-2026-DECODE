package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.rbrabson.behave.BehaviorTree;
import com.rbrabson.behave.Node;
import com.rbrabson.behave.Parallel;
import com.rbrabson.behave.Sequence;
import com.rbrabson.behave.Status;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions.FollowPath;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions.IntakeArtifacts;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions.ShootArtifacts;

/**
 * This class defines the autonomous pathing for the blue far starting position. It scores three
 * preloaded artifacts, then goes to the warehouse to pick up and score four more artifacts, then
 * ends in the storage unit.
 */
public class BlueFarPathing implements AutonomousPathing {
    public static final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private static final Pose shootPose = new Pose(47.84530386740332, 95.66850828729281, Math.toRadians(134));
    private static final Pose shootPose1 = new Pose(58.5, 13.723756906077352, Math.toRadians(118));
    private static final Pose pickup1Pose = new Pose(44.57458563535911, 83.07734806629837, Math.toRadians(180));
    private static final Pose pickup2Pose = new Pose(44.33149171270718, 58.83425414364642, Math.toRadians(180));
    private static final Pose pickup3Pose = new Pose(44.563535911602216, 35.005524861878456, Math.toRadians(180));
    private static final Pose pickup4Pose = new Pose(17.58011049723757, 10, Math.toRadians(180));
    private static final Pose intake1Pose = new Pose(25.96132596685084, 83.060773480663, Math.toRadians(180));
    private static final Pose intake2Pose = new Pose(25.386740331491723, 58.83425414364642, Math.toRadians(180));
    private static final Pose intake3Pose = new Pose(25.41436464088398, 35.97237569060775, Math.toRadians(180));
    private static final Pose intake4Pose = new Pose(12.596685082872927, 10, Math.toRadians(180));
    private static final Pose endPose = new Pose(32.03314917127072, 72.18232044198896, Math.toRadians(90));
    private static final Pose ctrl1 = new Pose(55.969613259668506, 48.85635359116024);
    private static final Pose ctrl2 = new Pose(64.58839779005525, 51.93646408839779);
    private static final Pose ctrl3 = new Pose(71.04972375690608, 30.70718232044198);
    private static final Pose ctrlGate = new Pose(55.12435743454239, 78.60429978380974);
    private static final Pose gate = new Pose(16.22651933701656, 78, Math.toRadians(90));

    private final BehaviorTree autonomousBehaviorTree;

    /**
     * Initializes the autonomous by building the paths and the behavior tree. This method should
     * be called once before run() is called repeatedly in a loop.
     *
     * @param follower The follower to use for following the paths. This follower will be reset
     *                 and set to follow the first path when this method is called.
     * @param intake   The intake mechanism to use for intaking artifacts. This will be used in
     *                 the behavior tree to run the intake when intaking artifacts.
     * @param shooter  The shooter mechanism to use for shooting artifacts. This will be used in
     *                 the behavior tree to run the shooter when shooting artifacts.
     * @param transfer The transfer mechanism to use for transferring artifacts from the intake to
     *                 the shooter.
     */
    public BlueFarPathing(@NonNull Follower follower, @NonNull Intake intake, @NonNull Shooter shooter, @NonNull Transfer transfer) {
        // Build the paths
        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();
        PathChain grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, pickup4Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), pickup4Pose.getHeading()) // was shootPose
                .build();
        PathChain intakePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, intake4Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), intake4Pose.getHeading())
                .build();
        PathChain scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, shootPose1))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), shootPose1.getHeading())
                .build();
        PathChain grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        PathChain intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .build();
        PathChain hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, ctrlGate, gate))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), gate.getHeading())
                .build();
        PathChain scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gate, shootPose))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose.getHeading())
                .build();
        PathChain grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrl2, pickup2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup2Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        PathChain intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .build();
        PathChain scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();
        PathChain grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrl3, pickup3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup3Pose.getHeading())
                .setBrakingStrength(1)
                .build();
        PathChain intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .build();
        PathChain scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shootPose1))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootPose1.getHeading()) // was pickup3Pose
                .build();
        PathChain ending = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, endPose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), endPose.getHeading())
                .build();

        // Build the behavior tree to run the paths
        Node root = new Sequence(
            new FollowPath(follower, scorePreload),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup4),
            new Parallel(
                new FollowPath(follower, intakePickup4),
                new IntakeArtifacts(intake)),
            new FollowPath(follower, scorePickup4),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup1),
            new Parallel(
                new FollowPath(follower, intakePickup1),
                new IntakeArtifacts(intake)),
            new FollowPath(follower, hitGate),
            new FollowPath(follower, scorePickup1),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup2),
            new Parallel(
                new FollowPath(follower, intakePickup2),
                new IntakeArtifacts(intake)),
            new FollowPath(follower, scorePickup2),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup3),
            new Parallel(
                new FollowPath(follower, intakePickup3),
                new IntakeArtifacts(intake)),
            new FollowPath(follower, scorePickup3),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, ending)
        );

        autonomousBehaviorTree = new BehaviorTree(root);
    }

    /**
     * Ticks the behavior tree to run the autonomous. Should be called repeatedly in
     * a loop until it returns SUCCESS or FAILURE.
     *
     * @return The status of the autonomous. Will return RUNNING until the
     *         autonomous is finished, at which point it will return either
     *         SUCCESS or FAILURE, depending on whether the autonomous
     *         completed successfully or not.
     */
    @Override
    public Status update() {
        return autonomousBehaviorTree.tick();
    }
}
