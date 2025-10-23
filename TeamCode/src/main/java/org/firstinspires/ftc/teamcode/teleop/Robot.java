package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
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

@TeleOp(name = "Clanker", group = "Orion")
public class Robot extends OpMode {

    public static Alliance alliance = Alliance.UNKNOWN;
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;
    VisionSubsystem visionSubsystem;
    FlywheelSubsystem flywheelSubsystem;

    public static Map<Integer, Artifact> motif = new HashMap<>();
    public static AtomicBoolean hasMotif = new AtomicBoolean(false);


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(telemetry, hardwareMap, gamepad1);
        driveSubsystem.init();

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem.init();

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1);
        shooterSubsystem.init();

        turretSubsystem = TurretSubsystem.getInstance(hardwareMap);
        turretSubsystem.init();

        visionSubsystem = VisionSubsystem.getInstance(hardwareMap);
        visionSubsystem.init();

        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap);
        flywheelSubsystem.init();


    }

    @Override
    public void loop() {
        driveSubsystem.loop();
        intakeSubsystem.loop();
        shooterSubsystem.loop();
        turretSubsystem.loop();
        visionSubsystem.loop();
        flywheelSubsystem.loop();
    }

}