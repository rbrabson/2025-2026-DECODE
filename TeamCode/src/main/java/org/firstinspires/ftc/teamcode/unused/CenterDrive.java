package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp
public class CenterDrive extends LinearOpMode {
    private Limelight3A limelight;

    // ADJUST THESE FOR YOUR ROBOT
    final double TARGET_DISTANCE = 12.0; // Need to change
    final double CAMERA_HEIGHT = 10.0;   // Measure from floor to camera lens center
    final double TAG_HEIGHT = 5.75;      // Measure from floor to apriltag center
    final double MOUNT_ANGLE = 15.0;     // Trial and error

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("flm");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("blm");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frm");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("brm");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, //Maybe need to change
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)); //Maybe need to change
        imu.initialize(parameters);


        limelight.pipelineSwitch(0); // Change to correct pipeline
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            double x, y, rx;

            LLResult result = limelight.getLatestResult();


            if (gamepad1.left_bumper && result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                //Distance calculations
                double angleToGoalRadians = Math.toRadians(MOUNT_ANGLE + ty);
                double currentDistance = (TAG_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
                double distanceError = currentDistance - TARGET_DISTANCE;

                //Tuning required
                y = distanceError * 0.04;  // Drive forward/back
                x = 0;
                rx = tx * 0.025;           // Turn to face tag


                if (Math.abs(tx) < 1.0) {
                    rx = 0; // Target reached
                } else {
                    rx = tx * 0.025; // Proportional turn
                }


                double minPower = 0.06; //also may need to change
                if (Math.abs(distanceError) < 0.5) {
                    y = 0; // Target reached
                } else {
                    y = distanceError * 0.04;
                    // Boost power if it's too weak to move the robot
                    if (Math.abs(y) < minPower) {
                        y = Math.signum(y) * minPower;
                    }
                }


                boolean distanceDone = (Math.abs(distanceError) < 0.5);
                boolean headingDone = (Math.abs(tx) < 1.0);

                if (distanceDone && headingDone) {
                    telemetry.addData("AUTO STATUS", "!!! TARGET LOCKED !!!");
                    gamepad1.rumble(500);
                } else {
                    telemetry.addData("AUTO STATUS", "Aligning...");
                }


                telemetry.addData("Mode", "AUTO ALIGNING");
                telemetry.addData("Distance", "%.2f in", currentDistance);
            } else {

                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;
                telemetry.addData("Mode", "FIELD CENTRIC");
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);

            telemetry.update();
        }
        limelight.stop();
    }
}
