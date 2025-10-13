package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static final String SERVO_NAME = "AngleServo";
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static final double GOAL_HEIGHT = 0.0;

    public static final double GEAR_RATIO = 2;
    public static final double TICKS_PER_REVOLUTION = 28.0;

    public static final double ENCODER_OFFSET = .2375;
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 360;

}
