package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionConstants;

public class ShooterSubsystem {
    private final ServoImplEx angleServo;
    private final Gamepad gamepad2;
    private double range;

    private Telemetry telemetry;

    // Initialize with at least 2 points for interpolation
    private final double[] calibDistances = {0.0, 100.0};
    private final double[] velocityResiduals = {0.0, 0.0};
    private final double[] angleResiduals = {0.0, 0.0};

//    private final VisionSubsystem vision = VisionSubsystem.getInstance();
    private final FlywheelSubsystem flywheelSubsystem = FlywheelSubsystem.getInstance();

    private static ShooterSubsystem instance;

    private double targetAngle = 0;
    private final Gamepad gamepad1;

    public ShooterSubsystem(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry, Gamepad gamepad1) {
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
        this.gamepad1 = gamepad1;
        this.angleServo = hardwareMap.get(ServoImplEx.class, ShooterConstants.SERVO_NAME);
    }

    public void init() {
        angleServo.getController().setServoPosition(angleServo.getPortNumber(), 0);
    }

    public void loop() {
        // Update multi-turn position tracking

//        if (!vision.getYDegrees().isPresent()) return;
//
//        double yDegrees = vision.getYDegrees().get();
//        double totalAngleDegrees = yDegrees + VisionConstants.CAM_ANGLE;
//        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

//        double heightDiff = ShooterConstants.GOAL_HEIGHT - VisionConstants.CAM_HEIGHT;

        // Protect against division by zero or negative tan
//        if (Math.tan(totalAngleRadians) <= 0) return;
//
//        range = heightDiff / Math.tan(totalAngleRadians);

//        if (gamepad2.left_bumper) {
//            shoot();
//        }

    }




    public void shoot() {
//        if (!vision.getYDegrees().isPresent()) return;
//
//        double theta = getTheta(ShooterConstants.NOMINAL_VELOCITY);
//        if (theta < 0) return;
//
//        double velocity = getVelocity(theta);
//        if (velocity < 0) return;
//
//        flywheelSubsystem.setVelocity(velocity);
//        setAngle(Math.toDegrees(theta));
    }

    /**
     * Get current shooter angle in degrees (approximate)
     */
    public double getAngleDegrees() {
        double servoPos = angleServo.getPosition(); // 0.0–1.0
        return ShooterConstants.MIN_ANGLE +
                servoPos * (ShooterConstants.MAX_ANGLE - ShooterConstants.MIN_ANGLE);
    }

    /**
     * Set shooter angle (direct mapping)
     */
    public void setAngle(double targetAngleDegrees) {
        targetAngleDegrees = Range.clip(targetAngleDegrees, ShooterConstants.MIN_ANGLE, ShooterConstants.MAX_ANGLE);

        double normalized = (targetAngleDegrees - ShooterConstants.MIN_ANGLE) /
                (ShooterConstants.MAX_ANGLE - ShooterConstants.MIN_ANGLE);

        angleServo.setPosition(normalized);
    }


    /**
     * Check if shooter is at target angle
     */
    public boolean atTargetAngle() {
        return Math.abs(targetAngle - getAngleDegrees()) < 1.0;
    }



    // === Residual interpolation helper ===
    private double interpolateResidual(double x, double[] xs, double[] ys) {
        if (xs.length == 0) return 0.0;
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];

        for (int i = 0; i < xs.length - 1; i++) {
            if (x >= xs[i] && x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return 0.0;
    }

    // === Velocity prediction with additive correction ===
    private double getVelocity(double theta) {
        // FIXED: Corrected projectile motion formula
        // v² = (g * r²) / (2 * cos²(θ) * (r*tan(θ) - Δh))
        double heightDiff = ShooterConstants.GOAL_HEIGHT - VisionConstants.CAM_HEIGHT;

        double numerator = 9.81 * Math.pow(range, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) *
                (range * Math.tan(theta) - heightDiff);

        if (denominator <= 0) return -1.0;

        double v_squared = numerator / denominator;
        if (v_squared <= 0) return -1.0;

        double v = Math.sqrt(v_squared);

        // Apply additive residual correction
        double residual = interpolateResidual(range, calibDistances, velocityResiduals);
        return v + residual;
    }

    // === Theta prediction with additive correction ===
//    private double getTheta(double velocity) {
////        if (!vision.getYDegrees().isPresent()) return -1.0;
////
////        // FIXED: Convert to radians and use tan
////        double yDegrees = vision.getYDegrees().get();
////        double totalAngleDegrees = yDegrees + VisionConstants.CAM_ANGLE;
////        double totalAngleRadians = Math.toRadians(totalAngleDegrees);
//
//        double heightDiff = ShooterConstants.GOAL_HEIGHT - VisionConstants.CAM_HEIGHT;
////        double calculatedRange = heightDiff / Math.tan(totalAngleRadians);
//
//        // Quadratic formula: A*tan²(θ) + B*tan(θ) + C = 0
//        double g = 9.81;
//        double A = -0.5 * g * Math.pow(calculatedRange, 2) / Math.pow(velocity, 2);
//        double B = calculatedRange;
//        double C = A - heightDiff;
//
//        double discriminant = Math.pow(B, 2) - 4 * A * C;
//        if (discriminant < 0) return -1.0;
//
//        double sqrtDisc = Math.sqrt(discriminant);
//        double tanTheta1 = (-B + sqrtDisc) / (2 * A);
//        double tanTheta2 = (-B - sqrtDisc) / (2 * A);
//
//        // Choose the higher angle (more arc) for typical shooting
//        double tanTheta = Math.max(tanTheta1, tanTheta2);
//        double theta = Math.atan(tanTheta);
//
//        // Apply additive residual correction
//        double residualDegrees = interpolateResidual(calculatedRange, calibDistances, angleResiduals);
//        return theta + Math.toRadians(residualDegrees);
//    }

    public static ShooterSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry, Gamepad gamepad1) {
        if (instance == null) {
            instance = new ShooterSubsystem(hardwareMap, gamepad2, telemetry, gamepad1);
        }
        return instance;
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ShooterSubsystem not initialized. Call getInstance(hardwareMap, gamepad2) first.");
        }
        return instance;
    }
}