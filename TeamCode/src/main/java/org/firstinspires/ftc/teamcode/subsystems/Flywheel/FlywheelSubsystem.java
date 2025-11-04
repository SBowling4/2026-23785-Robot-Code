package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.FeedForward;


public class FlywheelSubsystem {
    public MotorEx leftMotor;
    public MotorEx rightMotor;

    private FeedForward ff;
    private PIDController pid;

    private double lastTimeNs;
    public double lastTargetRadPerSec = 0.0;

    public double lastTargetVolts = 0.0;

    public VoltageSensor voltageSensor;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;

    public double tuningVelocity = 0.0;

    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
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

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        lastTimeNs = System.nanoTime();
    }

    public void loop() {

        if (Robot.tuningMode) {
            if (gamepad1.dpad_right) {
                tuningVelocity += 2.5;
            } else if (gamepad1.dpad_left) {
                tuningVelocity -= 2.5;
            }

            setVelocity(tuningVelocity);
        } else {
            if (gamepad1.left_bumper) {
                setVelocity(FlywheelConstants.CLOSE_VELOCITY);
            } else if (gamepad1.right_bumper) {
                setVelocity(FlywheelConstants.FAR_VELOCITY);
            } else {
                stop();
            }
        }





//        setVelocity(tuningVelocity);

        ff.setkS(FlywheelConstants.kS);
        ff.setkV(FlywheelConstants.kV);

        pid.setP(FlywheelConstants.kP);
        pid.setI(FlywheelConstants.kI);
        pid.setD(FlywheelConstants.kD);


    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
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

        setVoltage(- volts);
    }



    public void setVoltage(double volts) {
        double power = Range.clip(volts / getRobotVoltage(), -1.0, 1.0);
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public double getRobotVoltage() {
        if (voltageSensor != null) {
            return voltageSensor.getVoltage();
        } else {
            return 12;
        }
    }


    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap, gamepad1, telemetry);
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