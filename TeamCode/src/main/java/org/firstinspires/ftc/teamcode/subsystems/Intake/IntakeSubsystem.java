package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    private static IntakeSubsystem instance;

    MotorEx intakeMotor;

    HardwareMap hardwareMap;

    Gamepad gamepad1;

    private Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;

        intakeMotor = new MotorEx(hardwareMap, "IntakeMotor");
    }

    public void init() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop() {
        if (gamepad1.a) {
            intakeMotor.set(1);
        } else {
            intakeMotor.stopMotor();
        }

//        telemetry.addLine("//Intake//");
//        telemetry.addData("Intake Pressed", gamepad1.a);
//
//        telemetry.update();
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
