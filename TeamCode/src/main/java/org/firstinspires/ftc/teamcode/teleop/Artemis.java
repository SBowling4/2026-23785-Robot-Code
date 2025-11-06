package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "Artemis", group = "Orion")
public class Artemis extends OpMode {
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;
    VisionSubsystem visionSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;

    private boolean lastUpState = false;
    private boolean lastDownState = false;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(telemetry, hardwareMap, gamepad1);
        driveSubsystem.init();

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem.init();

        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        flywheelSubsystem.init();

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        shooterSubsystem.init();

        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem.init();

    }

    @Override
    public void loop() {
        driveSubsystem.loop();
        intakeSubsystem.loop();
        shooterSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();

        boolean currentUpState = gamepad1.dpad_up;
        boolean currentDownState = gamepad1.dpad_down;

        if (currentUpState && !lastUpState) {
            Robot.advanceState();
        }

        if (currentDownState && !lastDownState) {
            Robot.reverseState();
        }

        lastUpState = currentUpState;
        lastDownState = currentDownState;

        if (gamepad1.left_bumper) {
            flywheelSubsystem.setVelocity(Robot.shooterState.velocity);
            shooterSubsystem.setAngle(Robot.shooterState.angle);
        } else {
            flywheelSubsystem.stop();
            shooterSubsystem.setAngle(0);
        }



        telemetry.addLine("//Shooter//");
        telemetry.addData("Shooter State", Robot.shooterState.toString());
        telemetry.addLine();

//        telemetry.addData("Tuning Target Angle", shooterSubsystem.tuningPos);
        telemetry.addData("Target Angle", shooterSubsystem.targetPos);
        telemetry.addData("Current Angle", shooterSubsystem.getPosition());
        telemetry.addLine();

        telemetry.addData("Flywheel Velocity", flywheelSubsystem.getVelocity());
        telemetry.addData("Flywheel Target", flywheelSubsystem.lastTargetRadPerSec);
//        telemetry.addData("Tuning Flywheel Target", flywheelSubsystem.tuningVelocity);
        telemetry.addData("Flywheel Volts", flywheelSubsystem.lastTargetVolts);
        telemetry.addLine();



        telemetry.addLine("//Vision//");
        telemetry.addData("X", visionSubsystem.getXDegrees().isPresent() ? visionSubsystem.getXDegrees() : -1);
        telemetry.addData("Y", visionSubsystem.getYDegrees().isPresent() ? visionSubsystem.getYDegrees() : -1);



        telemetry.update();
    }

}