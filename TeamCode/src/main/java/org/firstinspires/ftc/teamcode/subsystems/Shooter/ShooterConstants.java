package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static final String SERVO_NAME = "AngleServo";
    public static final String LIMIT_SWITCH_NAME = "ShooterLS";
    public static double kP = 0.0649;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static final double GOAL_HEIGHT = 0.98;

    public static final double GEAR_RATIO = .5;
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 25.0;

    public static final double NOMINAL_VELOCITY = 200;
    public static final double FAR_ANGLE = 25;
    public static final double MID_ANGLE = 20.5;
    public static final double CLOSE_ANGLE = 6;

    public static final double SHOOTER_HEIGHT = .2; //Meters

}
