package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

/**
 * Class representing the turret mechanism of the robot. The turret is
 * controlled by a DC motor, which can be set to specific target positions based
 * on the desired angle.
 */
public class Turret {
    private static final int POSITION_CLOSE = 20;
    private static final double VELOCITY_LOW = 5;

    private static final int TURRET_OFFSET = 0;
    private static final int TURRET_MAX_LEFT = 1700 - TURRET_OFFSET;
    private static final int TURRET_MAX_RIGHT = -350 - TURRET_OFFSET;
    private static final double TURRET_MULTIPLIER = ((double) 742 / (double) 90);

    private static final double RUN_POWER = 1.0;

    private static final double DEFAULT_BX = 0;
    private static final double DEFAULT_BY = 144;

    private final DcMotorEx turret;
    private final Telemetry telemetry;

    private double baseX = DEFAULT_BX;
    private double baseY = DEFAULT_BY;

    private int targetPosition = 0;

    /**
     * Constructor for the Turret class. Initializes the turret motor and sets it to
     * the appropriate mode and direction.
     *
     * @param hardwareMap The hardware map to access the turret motor.
     * @param telemetry   The telemetry object to log initialization status and
     *                    target positions.
     */
    public Turret(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap");
        this.turret = map.get(DcMotorEx.class, "turret");
        this.telemetry = telemetry;

        initializeTurretMotor();

        telemetry.addLine("Turret initialized");
    }

    /**
     * Initializes the turret motor with the appropriate direction, mode, and
     * initial position.
     */
    private void initializeTurretMotor() {
        this.turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.0);
    }

    /**
     * Sets the base values for the turret's position calculations. These values are
     * used as the reference point for calculating the turret's target position
     * based on the target's coordinates.
     *
     * @param baseX The x-coordinate of the base position for the turret.
     * @param baseY The y-coordinate of the base position for the turret.
     */
    public void setBaseValues(double baseX, double baseY) {
        this.baseX = baseX;
        this.baseY = baseY;
    }

    /**
     * Calculates and sets the turret target position based on the given x, y
     * coordinates and robot heading. The method ensures that the calculated turret
     * position does not exceed the defined left and right limits.
     *
     * @param x       The x-coordinate of the target position.
     * @param y       The y-coordinate of the target position.
     * @param heading The current heading of the robot in radians.
     */
    public void setTargetPosition(double x, double y, double heading) {
        double targetAngleDeg = Math.toDegrees(Math.atan2(baseY - y, baseX - x));
        double robotHeadingDeg = Math.toDegrees(heading);
        double turretAngleDeg = normalizeDegrees(targetAngleDeg - (robotHeadingDeg -90.0));

        int turretTicks = (int) Math.round(turretAngleDeg * TURRET_MULTIPLIER);
        setTargetPosition(turretTicks);
    }

    /**
     * Sets the target position of the turret, adjusting for the offset.
     *
     * @param position The desired target position for the turret, which will be
     *                 adjusted by the TURRET_OFFSET.
     */
    public void setTargetPosition(int position) {
        int corrected = position - TURRET_OFFSET;
        targetPosition = Range.clip(corrected, TURRET_MAX_RIGHT, TURRET_MAX_LEFT);
        turret.setTargetPosition(targetPosition);
        turret.setPower(RUN_POWER);

        telemetry.addData("[TURRET] target ticks", targetPosition);
    }

    /**
     * Checks if the turret is within an acceptable error margin of the target position.
     *
     * @return True if the turret is at the target position, false otherwise.
     */
    public boolean isAtTarget() {
        boolean positionClose = Math.abs(getCurrentPosition() - targetPosition) < POSITION_CLOSE;
        boolean velocityLow = Math.abs(turret.getVelocity()) < VELOCITY_LOW;
        return positionClose && velocityLow;
    }

    /**
     * Returns the current position of the turret motor.
     *
     * @return The current encoder position of the turret motor.
     */
    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    /**
     * Normalizes an angle in degrees to the range [-180, 180]. This is used to ensure that the
     * turret angle calculations are consistent and do not exceed the defined limits.
     *
     * @param degrees The angle in degrees to be normalized.
     * @return The normalized angle in degrees, constrained to the range [-180, 180].
     */
    private static double normalizeDegrees(double degrees) {
        double d = degrees %360.0;
        if (d <= -180.0) d +=360.0;
        if (d >180.0) d -=360.0;
        return d;
    }
}
