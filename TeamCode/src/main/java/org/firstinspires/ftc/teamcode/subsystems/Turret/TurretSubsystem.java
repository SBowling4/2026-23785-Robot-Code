package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

import java.util.Optional;

public class TurretSubsystem {

    // Singleton instance
    private static TurretSubsystem instance;

    private final CRServoImplEx leftTurretServo;
    private final CRServoImplEx rightTurretServo;
    private final PIDController pidController;

    // multi-turn tracking
    private double revolutions = 0.0;
    private double lastRawPosition = 0.0;
    private double power = 0.0;

    private final VisionSubsystem vision = VisionSubsystem.getInstance();

    // Private constructor for singleton
    private TurretSubsystem(HardwareMap hardwareMap) {
        leftTurretServo = hardwareMap.get(CRServoImplEx.class, "LeftTurretServo");
        rightTurretServo = hardwareMap.get(CRServoImplEx.class, "RightTurretServo");
        pidController = new PIDController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD
        );

        // Initialize lastRawPosition to avoid false wrap on first read
        lastRawPosition = getRawPosition();
    }

    public void init() {}

    /**
     * Main loop: keep turret pointed at AprilTag.
     * Uses vision offset as an error signal (goal = 0°).
     */
    public void loop() {
        Optional<Double> errorOpt = vision.getXDegrees();

        if (errorOpt.isPresent()) {
            double error = errorOpt.get(); // how far off-center the tag is
            setCorrection(error);
        } else {
            // No tag detected → hold current angle
            stop();
        }
    }

    /**
     * Get the raw Axon CR position (0.0 to 1.0 per revolution).
     */
    private double getRawPosition() {
        ServoControllerEx controller = (ServoControllerEx) leftTurretServo.getController();
        return controller.getServoPosition(leftTurretServo.getPortNumber());
    }

    /**
     * Returns continuous position in servo rotations (multi-turn).
     */
    public double getContinuousPosition() {
        double raw = getRawPosition();

        // Detect wraparound
        if (raw - lastRawPosition > 0.5) {
            // jumped backwards across 0
            revolutions -= 1;
        } else if (lastRawPosition - raw > 0.5) {
            // jumped forwards across 1.0
            revolutions += 1;
        }

        lastRawPosition = raw;
        return revolutions + raw;
    }

    /**
     * Returns continuous turret angle in degrees, including gear ratio.
     */
    public double getContinuousAngleDegrees() {
        return getContinuousPosition() * 360.0 * TurretConstants.GEAR_RATIO;
    }

    /**
     * Run PID to correct turret offset (goal = 0° error).
     */
    private void setCorrection(double errorDegrees) {
        // PID drives error → 0
        power = pidController.calculate(0.0, errorDegrees);

        // Clip to valid CRServo range
        power = Range.clip(power, -1.0, 1.0);

        // Apply power, inverting right servo if necessary
        leftTurretServo.setPower(power);
        rightTurretServo.setPower(-power);
    }

    /**
     * Stop turret movement (hold current angle).
     */
    public void stop() {
        power = 0.0;
        leftTurretServo.setPower(0.0);
        rightTurretServo.setPower(0.0);
    }

    public static TurretSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new TurretSubsystem(hardwareMap);
        }
        return instance;
    }

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("TurretSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }

}
