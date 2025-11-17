package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

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
    private final Gamepad gamepad2;

    public double tuningVelocity = 0.0;

    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
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

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

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
            if (gamepad1.a && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
                setPower(1);
            }
//            if (gamepad2.a) {
//                setVelocity(FlywheelConstants.CLOSE_VELOCITY);
//            } else if (gamepad2.b) {
//                setVelocity(FlywheelConstants.MID_VELOCITY);
//            } else if (gamepad2.x) {
//                setVelocity(FlywheelConstants.FAR_VELOCITY);
//            }
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
        return -(leftMotor.getVelocity()  / FlywheelConstants.TICKS_PER_REVOLUTION) * 2 * Math.PI;
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

        setVoltage(-volts);
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - lastTargetRadPerSec) < 5;
    }



    public void setVoltage(double volts) {
        double power = Range.clip(volts / getRobotVoltage(), -1.0, 1.0);
        leftMotor.set(power);
        rightMotor.set(power);
    }

    /**
     * Equation obtained from here: <a href="https://docs.google.com/spreadsheets/d/1m6Tb_BewsEm0vuEWVIr-rKV5Jfy468Ui95xVuQbh-_I/edit?usp=sharing">Spreadsheet</a>
     *
     * @param distance distance (m) from target (Front of robot to base of goal)
     * @return Desired velocity for flywheel (rad/s)
     */
    public double findVelocity(double distance) {
        return 207 + 68.1 * distance + -24.3 * Math.pow(distance, 2) + 6.48 * Math.pow(distance, 3);
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


    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap, gamepad1, gamepad2);
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