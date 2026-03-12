package org.firstinspires.ftc.teamcode.unused;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
@TeleOp(name="Test 1 Servo")
public class ServoTester1 extends LinearOpMode{


    ElapsedTime controller = new ElapsedTime();


    public void runOpMode() {


        Servo servo1 = hardwareMap.get(Servo.class, "index");
        servo1.setPosition(0);
        double servoPos = 0;


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {


            if (gamepad1.right_bumper) {
                servoPos += 0.00005;
            }
            if (gamepad1.left_bumper) {
                servoPos -= 0.00005;
            }


            servo1.setPosition(servoPos);
            telemetry.addData("Servo Position", servo1.getPosition());
            telemetry.update();




        }
    }
}
