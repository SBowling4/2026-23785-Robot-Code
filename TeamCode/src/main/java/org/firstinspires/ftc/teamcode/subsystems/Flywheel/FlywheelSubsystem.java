package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.FeedForward;


public class FlywheelSubsystem {
    private final MotorEx leftMotor;
    private final MotorEx rightMotor;

    private final FeedForward ff;
    private final PIDController pid;

    private double lastTimeNs;
    private double lastTargetRPM = 0.0;

    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        leftMotor = new MotorEx(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new MotorEx(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        ff = new FeedForward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

        pid = new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    }

    public void init() {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lastTimeNs = System.nanoTime();
    }

    public void setVelocity(double targetRPM) {
        long nowNs = System.nanoTime();
        double dt = Math.max(1e-6, (nowNs - lastTimeNs) / 1e9);
        lastTimeNs = nowNs;

        double currentRPM = leftMotor.getVelocity() * 60  / FlywheelConstants.TICKS_PER_REVOLUTION;

        double accelRpm = (targetRPM - lastTargetRPM) / dt;
        lastTargetRPM = targetRPM;

        double targetRadPerSec = targetRPM / 60.0 * 2.0 * Math.PI;
        double targetRadPerSec2 = accelRpm / 60.0 * 2.0 * Math.PI;

        double ffVolts = ff.calculate(targetRadPerSec, targetRadPerSec2);
        double pidOutput = pid.calculate(currentRPM, targetRPM);

        double volts = ffVolts + pidOutput;
        double power = volts / 12.0;

        leftMotor.set(Range.clip(power, -1.0, 1.0));
        rightMotor.set(Range.clip(power, -1.0, 1.0));
    }

    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap);
        }
        return instance;
    }

    public static FlywheelSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("FlywheelSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }


}
