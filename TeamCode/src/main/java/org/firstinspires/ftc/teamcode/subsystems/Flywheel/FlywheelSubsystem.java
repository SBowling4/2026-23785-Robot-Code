package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class FlywheelSubsystem {
    public MotorEx leftMotor;
    public MotorEx rightMotor;
    private double lastTargetRadPerSec = 0.0;

    public double lastTargetVolts = 0.0;
    private Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        leftMotor = new MotorEx(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new MotorEx(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftMotor.setFeedforwardCoefficients(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
        rightMotor.setFeedforwardCoefficients(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

        leftMotor.setVeloCoefficients(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
        rightMotor.setVeloCoefficients(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
        lastTargetRadPerSec = targetRadPerSec;

        rightMotor.setVelocity(targetRadPerSec, AngleUnit.RADIANS);
        leftMotor.setVelocity(targetRadPerSec, AngleUnit.RADIANS);
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
