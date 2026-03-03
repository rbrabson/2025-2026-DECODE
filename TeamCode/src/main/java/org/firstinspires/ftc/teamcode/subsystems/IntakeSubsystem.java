package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Controls the intake motor that picks up balls from the field.
 *
 * The intake is a set of spinning wheels at the front of the robot.
 * When powered on, they pull balls from the field into the indexer.
 *
 * TODO: [Student] Describe how the intake physically works on our robot.
 *       What type of wheels/rollers does it use? How does a ball travel
 *       from the field, through the intake, and into the indexer?
 *
 * TODO: [Student] Why is the motor direction set to REVERSE in RobotHardware?
 *       What would happen if we didn't reverse it?
 */
public class IntakeSubsystem {

    private final DcMotor intakeMotor;

    /**
     * Creates an IntakeSubsystem.
     *
     * @param intakeMotor  the intake DcMotor from RobotHardware
     */
    public IntakeSubsystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    /**
     * Turns the intake on or off.
     *
     * @param on  true to spin the intake at full power, false to stop
     */
    public void run(boolean on) {
        intakeMotor.setPower(on ? 1.0 : 0.0);
    }

    /**
     * Runs the intake at a specific power level.
     * Useful for running the intake in reverse to eject jammed balls.
     *
     * @param power  motor power from -1.0 (reverse) to 1.0 (forward)
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}
