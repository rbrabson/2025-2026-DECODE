package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PedroPathingDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.sensors.Limelight;

import java.util.List;

/**
 * The Robot class serves as a centralized hub for all the robot's mechanisms and subsystems.
 * It follows the singleton pattern to ensure that only one instance of the robot exists throughout the program.
 */
public class Robot{
    private static Robot robot;

    // Public fields for each mechanism and subsystem of the robot.
    public final MecanumDrive mecanumDrive;
    public final PedroPathingDrive pedroPathingDrive;
    public final Intake intake;
    public final Shooter shooter;
    public final Transfer transfer;
    public final Limelight limelight;
    public final Light light;
    public final VoltageSensor voltageSensor;

    public final List<LynxModule> allHubs;

    /**
     * Private constructor to initialize all the robot's mechanisms and subsystems.
     *
     * @param hardwareMap The hardware map used to initialize the robot's components.
     * @param telemetry   The telemetry object used for debugging and feedback during operation.
     */
    private Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = new Limelight(hardwareMap, telemetry);
        // mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        mecanumDrive = null; // Using PedroPathing for Teleop instead of MecanumDrive
        pedroPathingDrive = new PedroPathingDrive(hardwareMap, limelight.getSensor(), telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        light = new Light(hardwareMap, telemetry);
        voltageSensor = new VoltageSensor(hardwareMap, telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);
    }

    /**
     * Static method to get the single instance of the Robot class. If the instance does not exist,
     * it creates one.
     *
     * @param hardwareMap The hardware map used to initialize the robot's components.
     * @param telemetry   The telemetry object used for debugging and feedback during operation.
     * @return The single instance of the Robot class.
     */
    @NonNull
    public static Robot getInstance(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        if (robot == null) {
            robot = new Robot(hardwareMap, telemetry);
        }
        return robot;
    }

    /**
     * Static method to reset the Robot instance. This can be useful between runs of OpModes to
     * ensure that the hardware is read correctly at the start.
     */
    public static void reset() {
        robot = null;
    }

}
