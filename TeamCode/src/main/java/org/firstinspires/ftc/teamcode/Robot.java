package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

@Configurable
public class Robot {
    public static final Alliance alliance = Alliance.BLUE;

    public static Map<Integer, Artifact> motif = new HashMap<>();
    public static AtomicBoolean hasMotif = new AtomicBoolean(false);
    public static boolean tuningMode = false;

    public static ShooterStates shooterState = ShooterStates.MID;


    public enum ShooterStates {
        FAR(ShooterConstants.FAR_ANGLE, FlywheelConstants.FAR_VELOCITY),
        MID(ShooterConstants.MID_ANGLE, FlywheelConstants.MID_VELOCITY),
        CLOSE(ShooterConstants.CLOSE_ANGLE, FlywheelConstants.CLOSE_VELOCITY);

        public final double angle;
        public final double velocity;

        private ShooterStates(double angle, double velocity) {
            this.angle = angle;
            this.velocity = velocity;
        }
    }

    public static void advanceState() {
        switch (shooterState) {
            case FAR:
                break;
            case MID:
                shooterState = ShooterStates.FAR;
                break;
            case CLOSE:
                shooterState = ShooterStates.MID;
                break;
        }
    }

    public static void reverseState() {
        switch (shooterState) {
            case FAR:
                shooterState = ShooterStates.MID;
                break;
            case MID:
                shooterState = ShooterStates.CLOSE;
                break;
            case CLOSE:
                break;
        }
    }


}
