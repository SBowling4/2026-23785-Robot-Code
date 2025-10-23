package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FeedForward;


public class FlywheelSubsystem {
    public MotorEx leftMotor;
    public MotorEx rightMotor;

    private FeedForward ff;
    private PIDController pid;

    private double lastTimeNs;
    private double lastTargetRadPerSec = 0.0;

    public double lastTargetVolts = 0.0;

    public VoltageSensor voltageSensor;
    private Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        leftMotor = new MotorEx(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new MotorEx(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }

        ff = new FeedForward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

        pid = new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lastTimeNs = System.nanoTime();
    }

    public void loop() {
        telemetry.addLine("//Flywheel//");
        telemetry.addData("Velocity (rad/s)", "%.2f", getVelocity());
        telemetry.addData("Target Velocity (rad/s)", "%.2f", lastTargetRadPerSec);
        telemetry.addData("Last Target Volts", "%.2f", lastTargetVolts);
        telemetry.addLine();
        telemetry.addLine();


        telemetry.update();

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
        double pidVolts = pid.calculate(currentRadPerSec, targetRadPerSec);

        double volts = ffVolts + pidVolts;
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
