package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FeedForward;


public class FlywheelSubsystem {
    public final MotorEx leftMotor;
    public final MotorEx rightMotor;

    private final FeedForward ff;
    private final PIDController pid;

    private double lastTimeNs;
    private double lastTargetRadPerSec = 0.0;

    public double lastTargetVolts = 0.0;

    public VoltageSensor voltageSensor;

    private Telemetry telemetry;

    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = new MotorEx(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new MotorEx(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }

        this.telemetry = telemetry;

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

    public void loop() {
        getFf().setkS(FlywheelConstants.kS);
        getFf().setkV(FlywheelConstants.kV);

        pid.setP(FlywheelConstants.kP);
        pid.setI(FlywheelConstants.kI);
        pid.setD(FlywheelConstants.kD);
    }

    public double getVelocity() {
        return (leftMotor.getVelocity()  / FlywheelConstants.TICKS_PER_REVOLUTION) * 2 * Math.PI;
    }

    public void setVelocity(double targetRadPerSec) {
        long nowNs = System.nanoTime();
        double dt = Math.max(1e-6, (nowNs - lastTimeNs) / 1e9);
        lastTimeNs = nowNs;

        double currentRadPerSec = getVelocity();

        double accelRadPerSec2 = (targetRadPerSec - lastTargetRadPerSec) / dt;
        lastTargetRadPerSec = targetRadPerSec;


        double ffVolts = ff.calculate(targetRadPerSec);
        double pidOutput = pid.calculate(currentRadPerSec, targetRadPerSec);

        double volts = ffVolts + pidOutput;
        lastTargetVolts = volts;
        double power = volts / voltageSensor.getVoltage();

        leftMotor.set(Range.clip(power, -1.0, 1.0));
        rightMotor.set(Range.clip(power, -1.0, 1.0));
    }



    public void setVoltage(double volts) {
        double power = Range.clip(volts / getRobotVoltage(), -1.0, 1.0);
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public double getRobotVoltage() {
        if (voltageSensor != null) {
            return voltageSensor.getVoltage();
        } else {
            return FlywheelConstants.NOMINAL_VOLTAGE;
        }
    }

    public FeedForward getFf() {
        return ff;
    }

    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap, telemetry);
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
