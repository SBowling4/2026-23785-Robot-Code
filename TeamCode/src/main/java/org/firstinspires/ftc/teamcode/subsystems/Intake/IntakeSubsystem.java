package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private static IntakeSubsystem instance;

    MotorEx intakeMotor;

    HardwareMap hardwareMap;

    Gamepad gamepad1;

    public IntakeSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        intakeMotor = new MotorEx(hardwareMap, IntakeConstants.INTAKE_MOTOR_NAME);
    }

    public void loop() {
        if (gamepad1.a) {
            intakeMotor.set(1);
        } else {
            intakeMotor.stopMotor();
        }
    }

    public static IntakeSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad2) {
        if (instance == null) {
            instance = new IntakeSubsystem(hardwareMap, gamepad2);
        }
        return instance;
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("IntakeSubsystem not initialized. Call getInstance(hardwareMap, gamepad2) first.");
        }
        return instance;
    }
}
