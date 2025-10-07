package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

@Config
@TeleOp(name="Shooter Tuning")
public class ShooterTuning extends OpMode {
    ShooterSubsystem shooterSubsystem;
    FlywheelSubsystem flywheelSubsystem;

    double targetAngle = 0;

    @Override
    public void init() {
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap);
        flywheelSubsystem.init();

        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, telemetry, gamepad1);
        shooterSubsystem.init();
    }

    @Override
    public void loop() {
        shooterSubsystem.loop();

        if (gamepad1.dpad_up) {
            targetAngle += 0.01;
        } else if (gamepad1.dpad_down) {
            targetAngle -= 0.01;
        }

        shooterSubsystem.setAngle(targetAngle);


        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Current Angle", shooterSubsystem.getAngleDegrees());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current Angle", shooterSubsystem.getAngleDegrees());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.update();
    }
}
