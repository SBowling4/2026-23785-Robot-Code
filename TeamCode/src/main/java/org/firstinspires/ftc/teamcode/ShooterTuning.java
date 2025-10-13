package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;

@TeleOp
public class ShooterTuning extends OpMode {
    ShooterSubsystem shooterSubsystem;

    double targetAngle;

    @Override
    public void init() {
        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad2);
        shooterSubsystem.init();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            targetAngle += .25;
        } else if (gamepad1.dpad_down) {
            targetAngle -= .25;
        }

        shooterSubsystem.setAngle(targetAngle);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Angle Error", Math.abs(targetAngle - shooterSubsystem.getPosition()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        telemetry.addData("Current Angle", shooterSubsystem.getPosition());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Raw Angle", shooterSubsystem.encoder.getRevolutions());

        telemetry.update();
    }
}
