package org.firstinspires.ftc.teamcode.subsystems.Feeder;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FeederSubsystem {
    private MotorEx feederMotor;
    private static FeederSubsystem instance;

    private ColorRangeSensor colorSensor;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    public FeederSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        feederMotor = new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME);
        colorSensor = hardwareMap.get(ColorRangeSensor.class, FeederConstants.COLOR_SENSOR_NAME);
    }

    public void loop() {
        if (gamepad1.a) {
            feed();
        } else if (gamepad1.y) {
            back();
        } else {
            stop();
        }
    }

    public void feed() {
        feederMotor.set(1);
    }

    public void back() {
        feederMotor.set(-1);
    }

    public void stop() {
        feederMotor.stopMotor();
    }

    public boolean hasPiece() {
        return getDistance() < .5;
    }

    public double getDistance() {
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public static FeederSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new FeederSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static FeederSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Call getInstance(HardwareMap hardwareMap) first");
        }
        return instance;
    }


}
