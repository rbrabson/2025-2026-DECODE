package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Controller extends LinearOpMode {
    DcMotorEx motor;
    double Kp = 0; // Proportional constant
    double Ki = 0; // Integral constant
    double Kd = 0; // derivative constant
    double reference = 1500; // velocity that we want to reach
    double integralSum = 0; // this is the integral sum used to calculate error over time
    double lastError = 0; // used to figure out the change in error for derivative value

    /*
    Variable not currently used they are there for improvements

    double lastreference = reference; // to reset the integral sum
    double maxIntegralSum = 0; // this is set as a limit to prevent the system not being controlled by anything
    double a = 0.5; //used as a constant to fix the high frequency data TUNE this
    double previousFilterEstimate = 0; // used in low pass filter
    double currentFilterEstimate = 0; // used in low pass filter
     */
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "Motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpadUpWasPressed()){
                Kp+=0.01;
            }
            if(gamepad1.dpadDownWasPressed()){
                Kp-=0.01;
            }
            if (gamepad1.dpadRightWasPressed()){
                Ki+=0.01;
            }
            if (gamepad1.dpadLeftWasPressed()){
                Ki-=0.01;
            }
            if (gamepad1.aWasPressed()){
                Kd-=0.01;
            }
            if (gamepad1.yWasPressed()){
                Kd+=0.01;
            }
            telemetry.addData("KP", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
            double power = PIDControl();
            motor.setPower(power);

        }
    }

    public double PIDControl() {

        // obtain the encoder position
        double encoderPosition = motor.getCurrentPosition(); //

        // calculate the error
        double error = reference - encoderPosition; // calculating error

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum += (error * timer.seconds());


        /* POTENTIAL IMPROVEMENTS
         double errorChange = (error - lastError); // used in low pass filter

        // filter out high frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
       previousFilterEstimate = currentFilterEstimate;




        // max out integral sum
        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon setpoint changes
        if (reference != lastreference) {
            integralSum = 0;
        }


         lastreference = reference;

         */

        // formula to output motor power
        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        lastError = error;

        // reset the timer for next time
        timer.reset();
        return out;
    }
}
