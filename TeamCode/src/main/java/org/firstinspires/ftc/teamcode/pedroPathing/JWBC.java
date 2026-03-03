package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawPoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;


@Autonomous(name="JWBC")
@Disabled
public class JWBC extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;// Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose startPose = new Pose(33.524861878453045, 135.69060773480663, Math.toRadians(90));
    private final Pose ShootPose = new Pose(47.84530386740332, 95.66850828729281, Math.toRadians(134)); //Try to implement april tag locking for heading and
    private final Pose pickup1Pose = new Pose(44.57458563535911, 90.07734806629837, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(44.33149171270718, 65.674033149171294, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(44.563535911602216, 43.005524861878456, Math.toRadians(180));
    private final double BlueHoodX = 14;
    private final double BlueHoodY = 129;
    double formula = 0;


    private final Pose intake1Pose = new Pose(25.96132596685084, 90.060773480663, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(25.386740331491723, 65.83425414364642, Math.toRadians(180));
    private final Pose intake3Pose = new Pose(25.41436464088398, 43.97237569060775, Math.toRadians(180));
    private final Pose endpose = new Pose(32.03314917127072, 72.18232044198896, Math.toRadians(90));


    private final Pose Ctrl1 = new Pose(65.1767955801105, 82.73480662983425);
    private final Pose Ctrl2 = new Pose(64.58839779005525, 51.93646408839779);
    private final Pose Ctrl3 = new Pose(71.04972375690608, 30.70718232044198);


    private final Pose CtrlGate = new Pose(61.12435743454239, 78.60429978380974);


    private final double Bx = 0;
    private final double By = 144;
    private final double m = ((double) 742 / (double) 90);
    private Limelight3A limelight;
    boolean motifDetected = false;
    // Variable Initialization
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, turret;
    private DcMotorEx shooter1;
    private Servo hoodExtension, indexer, hinge;


    private DcMotor intake;


    private double increment = 0.2085;
    private double pos1Intake = 0.6708; // .9
    private double pos2Intake = 0.4695; // .7499
    private double pos3Intake = 0.2651; // .5404
    private double pos1Shoot = 0.9829; // .6444
    private double pos2Shoot = 0.7803; // .4381
    private double pos3Shoot = 0.5706; // .2331k≥≥≥≥≥≥≥≥≥≥≥≥
    private double TurretPosition = 0; // may need to change
    private int turretExtremeLeft = 1700; // may need to change
    private int turretExtremeRight = -350; // may need to change
    private String motif = "";
    private String pattern = "GPP";
    private Boolean goingLeft = true;
    private boolean track = false;
    private int turretPose = 0;
    private double kP = 5;
    private boolean resetPattern = false;
    private static boolean shooting = false;
    private boolean shooting2 = false;
    private int indexerState = 0;
    private int iteration = 0;
    private boolean stopShooting = true;
    private boolean shoot = false;
    public int comparison = 0;
    boolean centered = false;
    boolean centerControl = false;
    boolean turret123 = false;
    boolean intakeBool = false;
    double hoodPos = 0;


    PredominantColorProcessor colorSensor;


    // Elapsed Times
    ElapsedTime rightTriggerDuration = new ElapsedTime();
    ElapsedTime intakeDelay = new ElapsedTime();
    ElapsedTime rightBumperDuration = new ElapsedTime();
    ElapsedTime leftTriggerDuration = new ElapsedTime();
    ElapsedTime indexerTime = new ElapsedTime();
    ElapsedTime hingeTime = new ElapsedTime();
    ElapsedTime indexerTime2 = new ElapsedTime();
    ElapsedTime hingeTime2 = new ElapsedTime();
    ElapsedTime turretInterval = new ElapsedTime();


    ElapsedTime xTime = new ElapsedTime();
    ElapsedTime bTime = new ElapsedTime();
    ElapsedTime yTime = new ElapsedTime();
    ElapsedTime colorTime = new ElapsedTime();
    ElapsedTime leftTrigger = new ElapsedTime();
    ElapsedTime hinge12 = new ElapsedTime();
    ElapsedTime hinge22 = new ElapsedTime();
    ElapsedTime tracked = new ElapsedTime();



   private boolean flag = true;


    public static int count(String str, Character targetChar) {
        int iter = 0;
        for (int i = 0; i < str.length(); i++) {
            if (str.charAt(i) == targetChar) {
                iter++;
            }
        }
        return iter;
    }


    public void runIntake(boolean bool) {
        if (bool) {
            intake.setPower(1); // May need to change direction
        } else {
            intake.setPower(0);
        }
    }

    public void turretTracker(boolean track) {
        if (!track) return;

        if (motifDetected) {
            double targetAngleDeg = ((Math.toDegrees(Math.atan((By - follower.getPose().getY()) / (Bx-follower.getPose().getX()))) % 180) + 180) % 180;
            double robotHeadingDeg = Math.toDegrees(follower.getHeading());
            double turretAngleDeg = targetAngleDeg - (robotHeadingDeg - 90);
            turretPose = (int) (turretAngleDeg * m);

            if (turretPose > turretExtremeLeft || turretPose < turretExtremeRight) {
                return;
            }
        } else {
            // This will only execute once during first iteration of loop
            double targetAngleDeg = ((Math.toDegrees(Math.atan((144 - follower.getPose().getY()) / (72 - follower.getPose().getX()))) % 180) + 180) % 180;;
            double robotHeadingDeg = Math.toDegrees(follower.getHeading());
            double turretAngleDeg = targetAngleDeg - (robotHeadingDeg - 90);
            turretPose = (int) (turretAngleDeg * m);

            if (turretPose > turretExtremeLeft || turretPose < turretExtremeRight) {
                return;
            }
        }

        LLResult result1 = limelight.getLatestResult();

        double kP = 9;          // tune this
        double deadband = 2;    // degrees// encoder ticks per loop

        if (result1 != null && result1.isValid()) {

            telemetry.addData("Error", result1.getTx());
            double error = result1.getTx();
            if (Math.abs(error) < deadband) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }
            if (!motifDetected) {
                // This will only execute once during first iteration of loop and will determine the motif
                List<LLResultTypes.FiducialResult> fiducials = result1.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    int tagId = fiducials.get(0).getFiducialId();
                    if (tagId == 21) {
                        motif = "GPP";
                    } else if (tagId == 22) {
                        motif = "PGP";
                    } else if (tagId == 23) {
                        motif = "PPG";
                    }
                    //SharedClass.motif = motif;
                    limelight.pipelineSwitch(1);
                    motifDetected = true;
                }
            }
        }
        turret.setTargetPosition(turretPose);
    }

    public void automated_shoot(boolean launch) {
        char green = 'G';
        char purple = 'P';
        if (launch) {
            shooting = true;
            stopShooting = false;// May have to change direction
            if (count(pattern, green) == 1 && count(pattern, purple) == 2 && !motif.isEmpty()) {
                int motifDetect = motif.indexOf(green);
                int patternDetect = pattern.indexOf(green);
                if (motifDetect == patternDetect) {
                    if (iteration == 0) {
                        if (!centerControl) {
                            indexer.setPosition(pos1Shoot);
                            if (indexerTime.milliseconds() > 1000) {
                                if (flag) {
                                    hinge.setPosition(0.4);
                                }
                                if (hingeTime.milliseconds() > 1175) {
                                    hinge.setPosition(0.09);
                                    flag = false;
                                    if (hinge12.milliseconds() > 1350) {
                                        iteration += 1;
                                        hingeTime.reset();
                                        indexerTime.reset();
                                        hinge12.reset();
                                        flag = true;
                                    }
                                }
                            }
                        } else {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 175) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 350) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }








                    } if (iteration == 1) {
                        indexer.setPosition(pos2Shoot);
                        if (indexerTime.milliseconds() > 150) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 325) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 500) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }
                    } if (iteration == 2) {
                        indexer.setPosition(pos3Shoot);
                        if (indexerTime.milliseconds() > 150) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 325) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 500) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    stopShooting = true;
                                    flag = true;
                                }
                            }
                        }
                    }
                } else if (motifDetect == (patternDetect+1)%3) { // may have to change condition
                    if (iteration == 0) {
                        if (!centerControl) {
                            indexer.setPosition(pos3Shoot);
                            if (indexerTime.milliseconds() > 1000) {
                                if (flag) {
                                    hinge.setPosition(0.4);
                                }
                                if (hingeTime.milliseconds() > 1175) {
                                    hinge.setPosition(0.09);
                                    flag = false;
                                    if (hinge12.milliseconds() > 1350) {
                                        iteration += 1;
                                        hingeTime.reset();
                                        indexerTime.reset();
                                        hinge12.reset();
                                        flag = true;
                                    }
                                }
                            }
                        } else {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 175) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 350) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }
                    } if (iteration == 1) {
                        indexer.setPosition(pos3Shoot+increment);
                        if (indexerTime.milliseconds() > 150) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 325) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 500) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }
                    } if (iteration == 2) {
                        indexer.setPosition(pos3Shoot+(increment*2));
                        if (indexerTime.milliseconds() > 150) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 325) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 500) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    stopShooting = true;
                                    flag = true;
                                }
                            }
                        }
                    }
                } else {
                    if (iteration == 0) {
                        if (!centerControl) {
                            indexer.setPosition(pos2Shoot);
                            if (indexerTime.milliseconds() > 1000) {
                                if (flag) {
                                    hinge.setPosition(0.4);
                                }
                                if (hingeTime.milliseconds() > 1175) {
                                    hinge.setPosition(0.09);
                                    flag = false;
                                    if (hinge12.milliseconds() > 1350) {
                                        iteration += 1;
                                        hingeTime.reset();
                                        indexerTime.reset();
                                        hinge12.reset();
                                        flag = true;
                                    }
                                }
                            }
                        } else {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 175) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 350) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }
                    } if (iteration == 1) {
                        indexer.setPosition(pos2Shoot+increment);
                        if (indexerTime.milliseconds() > 150) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 325) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 500) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }
                    } if (iteration == 2) {
                        indexer.setPosition(pos2Shoot+(increment*2));
                        if (indexerTime.milliseconds() > 150) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 325) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 500) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    stopShooting = true;
                                    flag = true;
                                }
                            }
                        }
                    }
                }








                if (stopShooting) {
                    iteration = 0;
                    indexerState = 0;
                    pattern = "XXX";
                    centerControl = false;
                    shooting = false;
                }








            } else {
                if (iteration == 0) {
                    if (!centerControl) {
                        indexer.setPosition(pos1Shoot);
                        if (indexerTime.milliseconds() > 1000) {
                            if (flag) {
                                hinge.setPosition(0.4);
                            }
                            if (hingeTime.milliseconds() > 1175) {
                                hinge.setPosition(0.09);
                                flag = false;
                                if (hinge12.milliseconds() > 1350) {
                                    iteration += 1;
                                    hingeTime.reset();
                                    indexerTime.reset();
                                    hinge12.reset();
                                    flag = true;
                                }
                            }
                        }
                    } else {
                        if (flag) {
                            hinge.setPosition(0.4);
                        }
                        if (hingeTime.milliseconds() > 175) {
                            hinge.setPosition(0.09);
                            flag = false;
                            if (hinge12.milliseconds() > 350) {
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                                hinge12.reset();
                                flag = true;
                            }
                        }








                    }
                }
                if (iteration == 1) {
                    indexer.setPosition(pos2Shoot);
                    if (indexerTime.milliseconds() > 150) {
                        if (flag) {
                            hinge.setPosition(0.4);
                        }
                        if (hingeTime.milliseconds() > 325) {
                            hinge.setPosition(0.09);
                            flag = false;
                            if (hinge12.milliseconds() > 500) {
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                                hinge12.reset();
                                flag = true;
                            }
                        }
                    }
                }
                if (iteration == 2) {
                    indexer.setPosition(pos3Shoot);
                    if (indexerTime.milliseconds() > 150) {
                        if (flag) {
                            hinge.setPosition(0.4);
                        }
                        if (hingeTime.milliseconds() > 325) {
                            hinge.setPosition(0.09);
                            flag = false;
                            if (hinge12.milliseconds() > 500) {
                                iteration += 1;
                                hingeTime.reset();
                                indexerTime.reset();
                                hinge12.reset();
                                stopShooting = true;
                                flag = true;
                            }
                        }
                    }
                }
                if (stopShooting) {
                    iteration = 0;
                    indexerState = 0;
                    pattern = "XXX";
                    centerControl = false;
                    shooting = false;
                }
            }
        } else {
            iteration = 0;
            stopShooting = true;
            resetPattern = false;
            hingeTime.reset();
            indexerTime.reset();
            hinge12.reset();
        }
        telemetry.addData("Iteration:", iteration);
    }

    private final Pose Gate = new Pose(16.22651933701656,78,Math.toRadians(90));

    private PathChain scorePreload, grabPickup1, intakePickup1, HittingGate ,scorePickup1, grabPickup2, intakePickup2, scorePickup2, grabPickup3, intakePickup3, scorePickup3, ending;
    private PathChain end;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootPose.getHeading())
                .build();


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(ShootPose, Ctrl1, pickup1Pose))
                .setLinearHeadingInterpolation(ShootPose.getHeading(), pickup1Pose.getHeading())
                .build();


        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, intake1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setBrakingStrength(0.5)
                .build();


        HittingGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, CtrlGate ,Gate))
                .setConstantHeadingInterpolation(Math.toRadians(90))
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
                .setBrakingStrength(0.5)
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
                .setBrakingStrength(0.5)
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


    // Setup a variable for each drive wheel to save power level for telemetry
    private boolean flag1 = true;
    private boolean flag2 = true;
    private boolean flag3 = true;
    private boolean flag4 = true;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (flag1) {
                        shooting = true;
                        flag1 = false;
                    }
                    setPathState(2);
                }
                break;
            case 2:
                if (!shooting) {
                    follower.followPath(grabPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(HittingGate);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (flag2) {
                        shooting = true;
                        flag2 = false;
                    }
                    setPathState(7);
                }
                break;
            case 7:
                if (!shooting) {
                    follower.followPath(grabPickup2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup2);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    if (flag3) {
                        shooting = true;
                        flag3 = false;
                    }
                    setPathState(11);
                }
                break;
            case 11:
                if (!shooting) {
                    follower.followPath(grabPickup3);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickup3);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    if (flag4) {
                        shooting = true;
                        flag4 = false;
                    }
                    setPathState(15);
                }
                break;
            case 15:
                if (!shooting) {
                    follower.followPath(ending);
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // Set basic color parameters
        char green = 'G';
        char purple = 'P';
        char x1 = 'X';

        // set position parameters
        SharedClass.xPos = follower.getPose().getX();
        SharedClass.yPos = follower.getPose().getY();
        SharedClass.yaw = follower.getPose().getHeading();
        SharedClass.motif = motif;
        SharedClass.turretPose = turret.getCurrentPosition();

        hoodExtension.setPosition(0);
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        //set the turret position and determine the motif, motif detection will happen only once
        turretTracker(true);
        shooter1.setVelocity(500);
        //shoot the balls
        automated_shoot(shooting);
        //start intake
        runIntake(true);

        // Reset latch once ball leaves ROI
        // start pattern = GPP
        // This will be always execute because pattern doesn't contain X
        if ((count(pattern, x1) == 0) && !centerControl && !motif.isEmpty()) {
            //adjust indexer position for each loop - indexer position will always be pos1Shoot because pattern will always be GPP
            if (count(pattern, green) == 1 && count(pattern, purple) == 2) {
                int motifGreenPosition = motif.indexOf(green);
                int patternGreenPosition = pattern.indexOf(green);
                if (motifGreenPosition == patternGreenPosition) {
                    indexer.setPosition(pos1Shoot);
                } else if (motifGreenPosition == (patternGreenPosition+1)%3) {
                    indexer.setPosition(pos3Shoot);
                } else {
                    indexer.setPosition(pos2Shoot);
                }
            } else {
                indexer.setPosition(pos1Shoot);
            }
            centerControl = true;
            leftTrigger.reset();
        }

        // This will be -1 at start as pattern = GPP
        indexerState = pattern.indexOf("X");

        // This block will never execute as pattern will always be GPP
        if (!shooting && !shooting2 && !centerControl && indexerState != -1) {
            switch (indexerState) {
                case 0:
                    indexer.setPosition(pos1Intake);
                    break;
                case 1:
                    indexer.setPosition(pos2Intake);
                    break;
                case 2:
                    indexer.setPosition(pos3Intake);
                    break;
            }
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            if (count(pattern, x1) > 0 && !shooting && !shooting2 && !centerControl) {
                if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN && colorTime.milliseconds() > 500) {
                    pattern =
                            pattern.substring(0, indexerState)
                                    + "G"
                                    + pattern.substring(indexerState + 1);
                    colorTime.reset();
                } else if (result.closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE && colorTime.milliseconds() > 500) {
                    pattern =
                            pattern.substring(0, indexerState)
                                    + "P"
                                    + pattern.substring(indexerState + 1);
                    colorTime.reset();
                }
            }
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Pattern", pattern);
        telemetry.addData("Best Match:", colorSensor.getAnalysis().closestSwatch);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(10);
        limelight.pipelineSwitch(0);
        limelight.start();

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, -0.5, 0.4, -0.8))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.WHITE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLUE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "logi"))
                .build();

        frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
        backRightMotor = hardwareMap.get(DcMotor.class, "brm");

        DcMotor light = hardwareMap.get(DcMotor.class, "l");
        light.setPower(1);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(160, 0, 0, 15)); // (160, 15)
        hoodExtension = hardwareMap.get(Servo.class, "s1");

        indexer = hardwareMap.get(Servo.class, "index");

        hinge = hardwareMap.get(Servo.class, "h");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        boolean intakeToggle = false;
        int loop = 0;
        int position = 0;
        boolean condition = false;
        int previousPosition = position;
        boolean positionControl = true;
        boolean ballSeen = false;
        boolean swit = false;
        boolean camCondition = true;
        boolean deletion = false;
        int counting = 0;

        String manual_shoot = "";

        hinge.setPosition(0.09);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        follower.activateAllPIDFs();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}






