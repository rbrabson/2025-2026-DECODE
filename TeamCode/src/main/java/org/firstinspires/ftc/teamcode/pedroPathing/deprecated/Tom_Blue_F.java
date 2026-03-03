package org.firstinspires.ftc.teamcode.pedroPathing.deprecated; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "Tom_Blue_F", group = "Tom")
@Disabled
public class Tom_Blue_F extends OpMode {
    private String ballPattern = "none";
    private boolean hasAligned = false;
    private Limelight3A limelight;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose ShootPose = new Pose(47.84530386740332, 95.66850828729281, Math.toRadians(134));
    private final Pose ShootPose1 = new Pose(58.5, 13.723756906077352,Math.toRadians(118));
    private final Pose pickup1Pose = new Pose(40.57458563535911, 84.07734806629837, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(41.33149171270718, 59.674033149171294, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(42.563535911602216, 35.005524861878456, Math.toRadians(180));
    private final Pose pickup4Pose = new Pose(17.58011049723757, 8.889502762430936, Math.toRadians(180));

    private final Pose intake1Pose = new Pose(23.96132596685084, 84.060773480663, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(24.386740331491723, 59.83425414364642, Math.toRadians(180));
    private final Pose intake3Pose = new Pose(24.41436464088398, 34.97237569060775, Math.toRadians(180));
    private final Pose intake4Pose = new Pose(8.596685082872927, 8.5, Math.toRadians(180));
    private final Pose endpose = new Pose(32.03314917127072, 72.18232044198896, Math.toRadians(90));

    private final Pose Ctrl1 = new Pose(55.969613259668506, 48.85635359116024);
    private final Pose Ctrl2 = new Pose(64.58839779005525, 51.93646408839779);
    private final Pose Ctrl3 = new Pose(71.04972375690608, 30.70718232044198);
    private final Pose Ctrl4 = new Pose(42.07734806629835, 10.207182320441992);


    private final Pose Gate = new Pose(15.22651933701656,72.3259668508287,Math.toRadians(90));

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