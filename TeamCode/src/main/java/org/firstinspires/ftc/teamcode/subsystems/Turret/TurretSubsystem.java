package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

import java.util.Optional;

public class TurretSubsystem {

    // Singleton instance
    private static TurretSubsystem instance;
    private CRServoImplEx turretServo;
    private Motor.Encoder encoder;
    private PIDController pidController;

    // multi-turn tracking
    private double revolutions = 0.0;
    private double lastRawPosition = 0.0;
    private double power = 0.0;

    private final VisionSubsystem vision = VisionSubsystem.getInstance();

    private final HardwareMap hardwareMap;

    private TurretSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        turretServo = hardwareMap.get(CRServoImplEx.class, TurretConstants.TURRET_SERVO_NAME);
        encoder = FlywheelSubsystem.getInstance().leftMotor.encoder;

        pidController = new PIDController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD
        );

        // Initialize lastRawPosition to avoid false wrap on first read
        lastRawPosition = getRawPosition();
        encoder.reset();
    }

    /**
     * Main loop: keep turret pointed at AprilTag.
     * Uses vision offset as an error signal (goal = 0°).
     */
    public void loop() {
        Optional<Double> errorOpt = vision.getHorizontalAngle();

        if (errorOpt.isPresent()) {
            double error = errorOpt.get(); // how far off-center the tag is
            setCorrection(error);
        } else {
            // No tag detected → hold current angle
            stop();
        }
    }

    private double getRawPosition() {
        return encoder.getPosition();
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
        turretServo.setPower(power);
    }

    /**
     * Stop turret movement (hold current angle).
     */
    public void stop() {
        power = 0.0;
        turretServo.setPower(0.0);
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
