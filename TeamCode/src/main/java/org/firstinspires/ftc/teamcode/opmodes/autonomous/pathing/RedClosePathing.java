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

import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions.IntakeArtifacts;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions.FollowPath;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.actions.ShootArtifacts;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

/**
 * This autonomous is for the red close starting position. It will score the
 * three preloads, then hit the gate, then score the three pickups, then end in the storage unit.
 */
public class RedClosePathing implements AutonomousPathing {
    public static final Pose startPose = new Pose(110.69613259668509, 135.6906077348066, Math.toRadians(90));
    private static final Pose shootPose = new Pose(96.57458563535913, 95.8674033149171, Math.toRadians(44));
    private static final Pose pickup1Pose = new Pose(102.62983425414362, 87.10497237569064, Math.toRadians(0));
    private static final Pose pickup2Pose = new Pose(100.75690607734808, 61.889502762430936, Math.toRadians(0));
    private static final Pose pickup3Pose = new Pose(101.21546961325969, 36.607734806629836, Math.toRadians(0));
    private static final Pose intake1Pose = new Pose(119.49723756906077, 87.09944751381215, Math.toRadians(0));
    private static final Pose intake2Pose = new Pose(120.10497237569061, 61.707182320441994, Math.toRadians(0));
    private static final Pose intake3Pose = new Pose(120.45856353591161, 36.68508287292818, Math.toRadians(0));
    private static final Pose endPose = new Pose(112.08287292817678, 72.17679558011054, Math.toRadians(90));
    private static final Pose ctrl1 = new Pose(78.71823204419888, 83.99999999999999);
    private static final Pose ctrl2 = new Pose(78.37292817679558, 59.17955801104973);
    private static final Pose ctrl3 = new Pose(76.40883977900553, 40.27624309392262);
    private static final Pose ctrlGate = new Pose(82.87564256545761, 67.39570021619026);
    private static final Pose gate = new Pose(127.3922651933702, 79.98895027624306, Math.toRadians(90));

    private final BehaviorTree autonomousBehaviorTree;

    /**
     * Initializes the autonomous by building the paths and the behavior tree. This
     * method should be called once before run() is called repeatedly in a loop.
     *
     * @param follower The follower to use for following the paths. This follower
     *                 will be reset and set to follow the first path when this
     *                 method is called.
     * @param intake   The intake mechanism to use for intaking artifacts. This will
     *                 be used in the behavior tree to run the intake when intaking
     *                 artifacts.
     * @param shooter  The shooter mechanism to use for shooting artifacts. This
     *                 will be used in the behavior tree to run the shooter when
     *                 shooting artifacts.
     * @param transfer The transfer mechanism to use for transferring artifacts from
     *                 the intake to the shooter.
     */
    public RedClosePathing(@NonNull Follower follower, @NonNull Intake intake, @NonNull Shooter shooter, @NonNull Transfer transfer) {
        // Build the paths
        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        PathChain grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup1Pose.getHeading())
                .build();
        PathChain intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).setBrakingStrength(0.5)
                .build();
        PathChain hitGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, ctrlGate, gate))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        PathChain scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gate, shootPose))
                .setLinearHeadingInterpolation(gate.getHeading(), shootPose.getHeading())
                .build();
        PathChain grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrl2, pickup2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup2Pose.getHeading())
                .build();
        PathChain intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, intake2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).setBrakingStrength(0.5)
                .build();
        PathChain scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();
        PathChain grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrl3, pickup3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickup3Pose.getHeading())
                .build();
        PathChain intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, intake3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180)).setBrakingStrength(0.5)
                .build();
        PathChain scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, shootPose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), shootPose.getHeading()) // was pickup3Pose
                .build();

        PathChain ending = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

        // Build the behavior tree to run the paths
        Node root = new Sequence(
            new FollowPath(follower, scorePreload),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup1),
            new Parallel(new FollowPath(follower, intakePickup1), new IntakeArtifacts(intake)),
            new FollowPath(follower, hitGate), new FollowPath(follower, scorePickup1),
            new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup2),
            new Parallel(new FollowPath(follower, intakePickup2), new IntakeArtifacts(intake)),
            new FollowPath(follower, scorePickup2), new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, grabPickup3),
            new Parallel(new FollowPath(follower, intakePickup3), new IntakeArtifacts(intake)),
            new FollowPath(follower, scorePickup3), new ShootArtifacts(follower, shooter, intake, transfer),

            new FollowPath(follower, ending));

        autonomousBehaviorTree = new BehaviorTree(root);
    }

    /**
     * Ticks the behavior tree to run the autonomous. Should be called repeatedly in
     * a loop until it returns SUCCESS or FAILURE.
     *
     * @return The status of the autonomous. Will return RUNNING until the
     *         autonomous is finished, at which point it will return either SUCCESS
     *         or FAILURE, depending on whether the autonomous completed
     *         successfully or not.
     */
    @Override
    public Status update() {
        return autonomousBehaviorTree.tick();
    }
}
