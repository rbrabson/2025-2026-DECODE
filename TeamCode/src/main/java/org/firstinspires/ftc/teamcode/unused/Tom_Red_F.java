package org.firstinspires.ftc.teamcode.unused; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "Tom_Blue_C", group = "Tom")
public class Tom_Red_F extends OpMode {
    private String ballPattern = "none";
    private boolean hasAligned = false;
    private Limelight3A limelight;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(88, 7, Math.toRadians(90));
    private final Pose ShootPose = new Pose(96.57458563535913, 95.8674033149171, Math.toRadians(52));
    private final Pose ShootPose1 = new Pose(84.13812154696132, 17.734806629834296,Math.toRadians(69));
    private final Pose pickup1Pose = new Pose(102.62983425414362, 83.10497237569064, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(100.75690607734808, 58.889502762430936, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(101.21546961325969, 34.607734806629836, Math.toRadians(0));
    private final Pose pickup4Pose = new Pose(125.70165745856353, 8.552486187845302, Math.toRadians(0));

    private final Pose intake1Pose = new Pose(119.49723756906077, 83.09944751381215, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(120.10497237569061, 58.707182320441994, Math.toRadians(0));
    private final Pose intake3Pose = new Pose(120.45856353591161, 34.68508287292818, Math.toRadians(0));
    private final Pose intake4Pose = new Pose(135.82872928176798, 8.685082872928163, Math.toRadians(0));
    private final Pose endpose = new Pose(112.08287292817678, 72.17679558011054, Math.toRadians(90));

    private final Pose Ctrl1 = new Pose(87.07734806629834, 45.80662983425415);
    private final Pose Ctrl2 = new Pose(78.37292817679558, 59.17955801104973);
    private final Pose Ctrl3 = new Pose(76.40883977900553, 40.27624309392262);
    private final Pose Ctrl4 = new Pose(100.0524861878453, 7.392265193370193);


    private final Pose Gate = new Pose(129.3922651933702,73.98895027624306,Math.toRadians(90));

    private PathChain scorePreload, grabPickup4, intakePickup4, scorePickup4,  grabPickup1, intakePickup1, HittingGate ,scorePickup1, grabPickup2, intakePickup2, scorePickup2, grabPickup3, intakePickup3, scorePickup3, ending;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose1.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl4,  pickup4Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup4Pose.getHeading())
                .build();
        intakePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, intake4Pose))
                .setLinearHeadingInterpolation(pickup4Pose.getHeading(), intake4Pose.getHeading())
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, ShootPose1))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(),ShootPose1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup1Pose.getHeading())
                .build();

        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        HittingGate = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, Gate))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(),Gate.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(Gate, ShootPose))
                .setLinearHeadingInterpolation(Gate.getHeading(), ShootPose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl2, pickup2Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup2Pose.getHeading())
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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                motifFinder(1);
                break;
            case 1:
                handleAlignAndShoot(2);
                break;
            case 2:
                follower.followPath(grabPickup4);
                setPathState(3);
            case 3:
                //Add the intake code
                setPathState(4);
            case 4:
                follower.followPath(intakePickup4);
                setPathState(5);
            case 5:
                follower.followPath(scorePickup4);
                setPathState(6);
            case 6:
                handleAlignAndShoot(7);
            case 7:
                follower.followPath(grabPickup1);
                setPathState(8);
                break;
            case 8:
                //SID Pretty please add intake code
                setPathState(9);
            case 9:
                follower.followPath(intakePickup1);
                setPathState(10);
                break;
            case 10:
                follower.followPath(HittingGate);
                setPathState(11);
                break;
            case 11:
                follower.followPath(scorePickup1);
                setPathState(12);
                break;
            case 12:
                handleAlignAndShoot(13);
                break;
            case 13:
                follower.followPath(grabPickup2);
                setPathState(14);
                break;
            case 14:
                //SID PRETTY PLEASE ADD INTAKE CODE
                setPathState(15);
                break;
            case 15:
                follower.followPath(intakePickup2);
                setPathState(16);
                break;
            case 16:
                follower.followPath(scorePickup2);
                setPathState(17);
                break;
            case 17:
                handleAlignAndShoot(18);
                break;
            case 18:
                follower.followPath(grabPickup3);
                setPathState(19);
                break;
            case 19:
                //SID PRETTY PLEASE ADD INTAKE CODE
                setPathState(20);
                break;
            case 20:
                follower.followPath(intakePickup3);
                setPathState(21);
                break;
            case 21:
                follower.followPath(scorePickup3);
                setPathState(22);
                break;
            case 22:
                handleAlignAndShoot(23);
                break;
            case 23:
                follower.followPath(ending);
                setPathState(-1);
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        follower = Constants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        follower.setStartingPose(startPose);
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }


    private void handleAlignAndShoot(int nextState) {
        LLResult result = limelight.getLatestResult();
        boolean foundTag20 = false;
        double targetTx = 0;

        // Scan for Tag 20
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                if (fr.getFiducialId() == 20) {
                    foundTag20 = true;
                    targetTx = fr.getTargetXDegrees();
                    break;
                }
            }
        }

        // Alignment Logic
        if (!hasAligned && foundTag20 && pathTimer.getElapsedTimeSeconds() < 1500) {
            if (Math.abs(targetTx) > 1000) {
                double currentHeading = follower.getPose().getHeading();
                double correctedHeading = currentHeading - Math.toRadians(targetTx);
                Pose perfectShootPose = new Pose(ShootPose.getX(), ShootPose.getY(), correctedHeading);

                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), perfectShootPose))
                        .setLinearHeadingInterpolation(currentHeading, correctedHeading)
                        .build());
            } else {
                hasAligned = true;
                actionTimer.resetTimer();
            }
        }

        else if (hasAligned || pathTimer.getElapsedTimeSeconds() >= 1500) {
            // SHOOTING CODE HERE

            if (actionTimer.getElapsedTimeSeconds() > 800) {
                //Stop shooting here
                hasAligned = false;
                setPathState(nextState);
            }
        }

    }
    private void motifFinder(int nextstate1) {
        if (pathTimer.getElapsedTimeSeconds() < 100) {
            follower.followPath(scorePreload);
        }


        LLResult res = limelight.getLatestResult();
        if (res != null && res.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = res.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducials) {
                long id = fr.getFiducialId();


                if (id == 21 || id == 22 || id == 23) {
                    ballPattern = "Pattern_ID_" + id;
                    gamepad1.rumble(200);
                }
            }
        }


        if (!follower.isBusy()) {
            setPathState(nextstate1);
        }
    }
}
