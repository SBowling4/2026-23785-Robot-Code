package org.firstinspires.ftc.teamcode.subsystems.Feeder;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FeederSubsystem {
    private CRServo leftFeederServo;
    private CRServo rightFeederServo;
    private Servo kickerServo;

    private static FeederSubsystem instance;

    public FeederSubsystem(HardwareMap hardwareMap) {
        this.leftFeederServo = hardwareMap.get(CRServo.class, FeederConstants.LEFT_FEEDER_SERVO_NAME);
        this.rightFeederServo = hardwareMap.get(CRServo.class, FeederConstants.RIGHT_FEEDER_SERVO_NAME);
        this.kickerServo = hardwareMap.get(Servo.class, FeederConstants.KICKER_SERVO_NAME);
    }

    public void init() {
        rightFeederServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeederServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void feed() {
        leftFeederServo.setPower(-1);
        rightFeederServo.setPower(1);
    }

    public void stop() {
        leftFeederServo.setPower(0);
        rightFeederServo.setPower(0);
    }

    public static FeederSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new FeederSubsystem(hardwareMap);
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
