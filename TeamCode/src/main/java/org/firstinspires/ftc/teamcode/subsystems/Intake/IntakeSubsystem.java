package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private static IntakeSubsystem instance;

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

    MotorEx intakeMotor;

    HardwareMap hardwareMap;

    Gamepad gamepad2;

    public IntakeSubsystem(HardwareMap hardwareMap, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        intakeMotor = new MotorEx(hardwareMap, "IntakeMotor");
    }

    public void loop() {
        if (gamepad2.a) {
            intakeMotor.set(1);
        } else {
            intakeMotor.stopMotor();
        }
    }
}
