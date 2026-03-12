













package org.firstinspires.ftc.teamcode.unused;




import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import android.util.Size;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;



@Disabled
@TeleOp(name="ColdJohnWickRed")
public class jwr extends LinearOpMode {




    private Follower follower;
    private final double Bx = 0;
    private final double By = 144;
    private double distance;
    private final double m = ((double) 742 / (double) 90);
    private Limelight3A limelight;
    boolean detection = false;
    // Variable Initialization
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, turret;
    private DcMotorEx shooter1;
    private Servo hoodExtension, indexer, hinge;




    private final double BlueHoodX = 14;
    private final double BlueHoodY = 129;
    private final double closeX = 48;
    private final double closeY = 96;
    double formula = 0;
    double fixedCloseDist = Math.sqrt(Math.pow(BlueHoodX-closeX, 2)+Math.pow(BlueHoodY-closeY, 2));




    private DcMotor intake;




    private double increment = 0.2085;
    private double pos1Intake = 0.6708; // .9
    private double pos2Intake = 0.4695; // .7499
    private double pos3Intake = 0.2651; // .5404
    private double pos1Shoot = 0.9829; // .6444
    private double pos2Shoot = 0.7803; // .4381
    private double pos3Shoot = 0.5706; // .2331k≥≥≥≥≥≥≥≥≥≥≥≥
    private double TurretPosition = 0; // may need to change
    private int turretExtremeLeft = 1700-SharedClass.turretPose; // may need to change
    private int turretExtremeRight = -350-SharedClass.turretPose; // may need to change
    private String motif = SharedClass.motif;
    private String pattern = "XXX";
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




        double targetAngleDeg = ((Math.toDegrees(Math.atan((144 - follower.getPose().getY()) / (144-follower.getPose().getX()))) % 180) + 180) % 180;
        double robotHeadingDeg = Math.toDegrees(follower.getHeading());
        double turretAngleDeg = targetAngleDeg - (robotHeadingDeg - 90);
        turretPose = (int) (turretAngleDeg * m);




        if (turretPose-SharedClass.turretPose > turretExtremeLeft || turretPose-SharedClass.turretPose < turretExtremeRight) {
            return;
        }




        LLResult result1 = limelight.getLatestResult();




        double kP = 9;          // tune this
        double deadband = 1;    // degrees// encoder ticks per loop




        if (result1 != null && result1.isValid()) {




            telemetry.addData("Error", result1.getTx());




            double error = result1.getTx();




            if (Math.abs(error) < deadband) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }




        }
        turret.setTargetPosition(turretPose-SharedClass.turretPose);
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




    public void runOpMode() {




        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(SharedClass.xPos, SharedClass.yPos, Math.toRadians(SharedClass.yaw)));




        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(10);
        limelight.pipelineSwitch(1);
        limelight.start();
















        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
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
      /*
      Hood:
          2 Motors shooter wheel
          1 Servo that controls hood extension
          Limelight camera attached on top of the hooded shooter
      Turret:
          2 Standard Servos to control the turret
      Indexer:
          Controlled by a Standard Servo
          Regular camera with color detection ability
      Hinge:
          Standard Servo
      Intake:
          1 continuous rotation melonbotics super servo
      Drivetrain:
          4 motors
      */




        hinge.setPosition(0.09);
        hoodExtension.setPosition(0);




        waitForStart();




        if (isStopRequested()) return;




        while (opModeIsActive()) {




            follower.update();




            char green = 'G';
            char purple = 'P';
            char x1 = 'X';




            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;




            distance = Math.sqrt(Math.pow(0-follower.getPose().getX(), 2) + (Math.pow(144-follower.getPose().getY(), 2)));




            if (gamepad1.options) {
                imu.resetYaw();
            }




            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);




            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);




            rotX = rotX * 1.1;  // Counteract imperfect strafing




            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;




            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);




            if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                turret123 = !turret123;
            }




            turretTracker(turret123);




            if ((gamepad1.yWasPressed() || gamepad2.yWasPressed())) {
                swit = !swit;
            }




            if (swit) {
                shooter1.setVelocity(1500);
                telemetry.addLine("High Vel");
            } else {
                shooter1.setVelocity(1170);
                telemetry.addLine("Low Vel");
            }




            hoodExtension.setPosition(hoodPos);
            if (gamepad1.dpad_left) {
                hoodPos -= 0.05;
            }
            if (gamepad1.dpad_right) {
                hoodPos += 0.05;
            }




            if ((gamepad1.aWasPressed() || gamepad2.aWasPressed()) && count(pattern, green) >= 1) {
                manual_shoot += "G";
            }
            if ((gamepad1.xWasPressed() || gamepad2.xWasPressed()) && count(pattern, purple) >= 1) {
                manual_shoot += "P";
            }
            //if (manual_shoot.isEmpty()) {
            //    indexerTime.reset();
            //    hingeTime.reset();
            //}




            if (!manual_shoot.isEmpty() && !shooting) {
                shooting2 = true;




                if (positionControl) {
                    position = pattern.indexOf(manual_shoot.charAt(0));
                    indexerTime2.reset();
                    hingeTime2.reset();
                    hinge22.reset();
                    centerControl = true;
                    positionControl = false;
                }




                if (position == 0) {
                    indexer.setPosition(pos1Shoot);




                    if (indexerTime2.milliseconds() > 500) {
                        if (flag) {
                            hinge.setPosition(0.4);
                        }
                        if (hingeTime2.milliseconds() > 675) {
                            hinge.setPosition(0.09);
                            flag = false;
                            if (hinge22.milliseconds() > 850) {
                                hingeTime2.reset();
                                indexerTime2.reset();
                                hinge22.reset();
                                flag = true;
                                condition = true;
                            }
                        }
                    }




                } else if (position == 1) {
                    indexer.setPosition(pos2Shoot);




                    if (indexerTime2.milliseconds() > 500) {
                        if (flag) {
                            hinge.setPosition(0.4);
                        }
                        if (hingeTime2.milliseconds() > 675) {
                            hinge.setPosition(0.09);
                            flag = false;
                            if (hinge22.milliseconds() > 850) {
                                hingeTime2.reset();
                                indexerTime2.reset();
                                hinge22.reset();
                                condition = true;
                                flag = true;
                            }
                        }
                    }




                } else if (position == 2) {
                    indexer.setPosition(pos3Shoot);




                    if (indexerTime2.milliseconds() > 500) {
                        if (flag) {
                            hinge.setPosition(0.4);
                        }




                        if (hingeTime2.milliseconds() > 675) {
                            hinge.setPosition(0.09);
                            flag = false;




                            if (hinge22.milliseconds() > 850) {
                                hingeTime2.reset();
                                indexerTime2.reset();
                                hinge22.reset();
                                condition = true;
                                flag = true;
                            }
                        }
                    }
                }




                if (condition) {
                    condition = false;
                    positionControl = true;
                    pattern =
                            pattern.substring(0, position)
                                    + "X"
                                    + pattern.substring(position + 1);




                    manual_shoot = manual_shoot.substring(1);
                }




                if (manual_shoot.isEmpty()) {
                    shooting2 = false;
                    centerControl = false;
                    intakeBool = true;
                }




            } else {
                hingeTime2.reset();
                indexerTime2.reset();
                hinge22.reset();
            }








            // HAVE TO FIX!!!!!!!!!!!!!!!!!




            if (!intakeBool) {
                intakeDelay.reset();
            }




            if ((gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed())) {
                shooting = true;
            }




            automated_shoot(shooting);
            if ((gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) && rightTriggerDuration.milliseconds() > 500) {
                intakeToggle = !intakeToggle;
                rightTriggerDuration.reset();
            }
            runIntake(intakeToggle);
            // Simple subset logic ends
            // Indexing Logic
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();




// Reset latch once ball leaves ROI




            if ((count(pattern, x1) == 0 || ((gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) && leftTrigger.milliseconds() > 500)) && !centerControl) {
                if (count(pattern, green) == 1 && count(pattern, purple) == 2) {
                    int motifDetect = motif.indexOf(green);
                    int patternDetect = pattern.indexOf(green);
                    if (motifDetect == patternDetect) {
                        indexer.setPosition(pos1Shoot);
                    } else if (motifDetect == (patternDetect+1)%3) {
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




            indexerState = pattern.indexOf("X");




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




                if (intakeDelay.milliseconds() > 750) {
                    intakeBool = false;
                }
                if (!intakeBool) {
                    if (count(pattern, x1) > 0 && !shooting && !shooting2 && !centerControl && !deletion) {
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




            }
            telemetry.addData("Pattern", pattern);
            telemetry.addData("Result:", result.closestSwatch);
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Hood Position", hoodPos);
            telemetry.addData("Current Robot X: ", follower.getPose().getX());
            telemetry.addData("Current Robot Y: ", follower.getPose().getY());
            telemetry.addData("Heading: ", Math.toDegrees(follower.getHeading()));
            telemetry.addData("Distance", distance);
            telemetry.update();




        }




    }
} // Have to check code again




/*
Close {
  Close --> Ta: 4.4 , Angle:
  Far --> Ta: 0.50-0.53, Angle: 0.05
}




Far {
  Ta
}
*/
