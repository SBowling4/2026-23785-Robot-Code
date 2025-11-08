package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

public class ShooterSubsystem {
    public CRServoImplEx servo;
    public Motor.Encoder encoder;
    private DigitalChannel limitSwitch;
    public PIDController pid;
    private double range;
    public double position;

    private FlywheelSubsystem flywheelSubsystem;
    private VisionSubsystem vision;

    // Initialize with at least 2 points for interpolation
    private final double[] calibDistances = {0.0, 100.0};
    private final double[] velocityResiduals = {0.0, 0.0};
    private final double[] angleResiduals = {0.0, 0.0};

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private static ShooterSubsystem instance;

    public double tuningPos = 0;
    public double targetPos = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        servo = hardwareMap.get(CRServoImplEx.class, ShooterConstants.SERVO_NAME);

        encoder = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, gamepad2).rightMotor.encoder;

        limitSwitch = hardwareMap.get(DigitalChannel.class, ShooterConstants.LIMIT_SWITCH_NAME);

        pid = new PIDController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD
        );

        double ticksPerRev = 8192;
        double degreesPerPulse = (360.0 * ShooterConstants.GEAR_RATIO) / ticksPerRev;

        encoder.setDistancePerPulse(degreesPerPulse);

        position = 0;
        encoder.reset();

        pid.setTolerance(1);

        flywheelSubsystem = FlywheelSubsystem.getInstance();
        vision = VisionSubsystem.getInstance();

        tuningPos = 0;
    }

    public void loop() {
        // Update multi-turn position tracking
        position = getPosition();

        if (getLS()) { // Active low
            encoder.reset();
            position = 0;
        }

        if (Robot.tuningMode) {
            if (gamepad1.dpad_up) {
                tuningPos += .5;
            } else if (gamepad1.dpad_down) {
                tuningPos -= .5;
            }

            tuningPos = Range.clip(tuningPos, 0, 25);

            setAngle(tuningPos);
        } else {

        }


        if (!vision.getVerticalAngle().isPresent()) return;

        double yDegrees = vision.getVerticalAngle().get();
        double totalAngleDegrees = yDegrees + VisionConstants.CAM_ANGLE;
        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

        double heightDiff = ShooterConstants.GOAL_HEIGHT - ShooterConstants.SHOOTER_HEIGHT;

//         Protect against division by zero or negative tan
        if (Math.tan(totalAngleRadians) <= 0) return;

        range = heightDiff / Math.tan(totalAngleRadians);

        if (gamepad2.left_bumper) {
            shoot();
        }

    }

    public void shoot() {
        if (vision.getDistance() == -1) return;

        double veloFromDistance = getVelocity(vision.getDistance());
        double angleFromDistance = getAngle(vision.getDistance());

        setAngle(angleFromDistance);
        flywheelSubsystem.setVelocity(veloFromDistance);
    }

    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) encoder.getPosition() / ticksPerRev;

        return revolutions * 360.0 * ShooterConstants.GEAR_RATIO;
    }

    public boolean getLS() {
        return !limitSwitch.getState();
    }

    public void setAngle(double targetAngle) {
        targetAngle = Range.clip(targetAngle, ShooterConstants.MIN_ANGLE, ShooterConstants.MAX_ANGLE);

        this.targetPos = targetAngle;

        double power = pid.calculate(getPosition(), targetAngle);
        servo.setPower(power);
    }

    public double getAngle(double distance) {
        return 2.51 + 21.1 * distance + -2.61 * Math.pow(distance, 2) + -1.13 * Math.pow(distance, 3);
    }

    public double getVelocity(double distance) {
        return 200 + 9.4 * distance + -58.5 * Math.pow(distance, 2) + 17 * Math.pow(distance, 3);
    }

    // === Residual interpolation helper ===
    private double interpolate(double x, double[] xs, double[] ys) {
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

    public static ShooterSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        if (instance == null) {
            instance = new ShooterSubsystem(hardwareMap, gamepad1, gamepad2);
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