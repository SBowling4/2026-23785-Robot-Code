package org.firstinspires.ftc.teamcode.subsystems.Indexer;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Artifact;
import org.firstinspires.ftc.teamcode.util.IndexerSlot;

import java.util.HashMap;
import java.util.Map;

public class IndexerSubsystem {

    // Singleton instance
    private static IndexerSubsystem instance;

    public static IndexerSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new IndexerSubsystem(hardwareMap);
        }
        return instance;
    }

    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("IndexerSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }

    private final MotorEx indexerMotor;
    private final PIDController pidController;
    private final ColorSensor colorSensor;

    private double position;
    private double color;

    private final IndexerSlot slot1 = new IndexerSlot(Artifact.NONE);
    private final IndexerSlot slot2 = new IndexerSlot(Artifact.NONE);
    private final IndexerSlot slot3 = new IndexerSlot(Artifact.NONE);

    private final Map<IndexerSlot.IndexerPositions, IndexerSlot> currentIndexerPositions = new HashMap<>();
    private final Map<IndexerSlot.IndexerPositions, IndexerSlot> lastIndexerPositions = new HashMap<>();

    private IndexerState state = IndexerState.MOTIF;

    // Private constructor for singleton
    private IndexerSubsystem(HardwareMap hardwareMap) {
        indexerMotor = new MotorEx(hardwareMap, IndexerConstants.MOTOR_NAME);
        colorSensor = hardwareMap.get(ColorSensor.class, "IndexerColorSensor");

        pidController = new PIDController(
                IndexerConstants.kP,
                IndexerConstants.kI,
                IndexerConstants.kD
        );

        // Optional: zero encoder
        indexerMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        }

    public void init() {
        pidController.setTolerance(10);
        setInitIndexerPositions();
    }

    public void loop() {
        position = indexerMotor.getCurrentPosition();

        if (state == IndexerState.MOTIF) {
            if (!Robot.hasMotif.get()) return;

            for (int i = 1; i <= 3; i++) {
                Artifact needed = Robot.motif.get(i);

                if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.SHOOTER).getArtifact() == needed) {
                    ShooterSubsystem.getInstance().shoot();
                    continue;
                }

                if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.LEFT).getArtifact() == needed) {
                    rotatePos120();
                    ShooterSubsystem.getInstance().shoot();
                    continue;
                }

                if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.INTAKE).getArtifact() == needed) {
                    rotateNeg120();
                    ShooterSubsystem.getInstance().shoot();
                    continue;
                }

                intakeEmpty();
            }
        }

        if (state == IndexerState.RAPID_FIRE) {
            if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.SHOOTER).getArtifact() != Artifact.NONE) {
                ShooterSubsystem.getInstance().shoot();
            } else if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.LEFT).getArtifact() != Artifact.NONE) {
                rotatePos120();
                ShooterSubsystem.getInstance().shoot();
            } else if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.INTAKE).getArtifact() != Artifact.NONE) {
                rotateNeg120();
                ShooterSubsystem.getInstance().shoot();
            }
        }
    }

    public void rotatePos120() {
        double target = position + 120;
        double power = pidController.calculate(position, target);
        power = Range.clip(power, -1.0, 1.0);

        indexerMotor.set(power);

        // Rotate slot map
        lastIndexerPositions.putAll(currentIndexerPositions);
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.INTAKE,
                lastIndexerPositions.get(IndexerSlot.IndexerPositions.SHOOTER));
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.LEFT,
                lastIndexerPositions.get(IndexerSlot.IndexerPositions.INTAKE));
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.SHOOTER,
                lastIndexerPositions.get(IndexerSlot.IndexerPositions.LEFT));
    }

    public void rotateNeg120() {
        double target = position - 120;
        double power = pidController.calculate(position, target);
        power = Range.clip(power, -1.0, 1.0);

        indexerMotor.set(power);

        // Rotate slot map
        lastIndexerPositions.putAll(currentIndexerPositions);
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.INTAKE,
                lastIndexerPositions.get(IndexerSlot.IndexerPositions.LEFT));
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.LEFT,
                lastIndexerPositions.get(IndexerSlot.IndexerPositions.SHOOTER));
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.SHOOTER,
                lastIndexerPositions.get(IndexerSlot.IndexerPositions.INTAKE));
    }

    public void removeShooter() {
        currentIndexerPositions.get(IndexerSlot.IndexerPositions.SHOOTER).setArtifact(Artifact.NONE);
    }

    public void intake() {
        color = colorSensor.blue();

        // TODO: determine what slot is at intake based on encoder position
        // TODO: use actual color logic to determine Artifact type

        currentIndexerPositions.get(IndexerSlot.IndexerPositions.LEFT).setArtifact(Artifact.GREEN); // placeholder
    }

    public void shoot() {
        // Non-blocking: check once, don’t spin forever
        if (!pidController.atSetPoint()) return;

        removeShooter();
        ShooterSubsystem.getInstance().shoot();
    }

    public void intakeEmpty() {
        if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.INTAKE).getArtifact() == Artifact.NONE) return;

        if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.SHOOTER).getArtifact() == Artifact.NONE) {
            rotatePos120();
            return;
        }

        if (currentIndexerPositions.get(IndexerSlot.IndexerPositions.LEFT).getArtifact() == Artifact.NONE) {
            rotateNeg120();
        }
    }

    public void updateState(IndexerState newState) {
        state = newState;
    }

    public void setInitIndexerPositions() {
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.INTAKE, slot1);
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.LEFT, slot2);
        currentIndexerPositions.put(IndexerSlot.IndexerPositions.SHOOTER, slot3);

        lastIndexerPositions.put(IndexerSlot.IndexerPositions.INTAKE, slot1);
        lastIndexerPositions.put(IndexerSlot.IndexerPositions.LEFT, slot3);
        lastIndexerPositions.put(IndexerSlot.IndexerPositions.SHOOTER, slot2);
    }

    public enum IndexerState {
        MOTIF,
        RAPID_FIRE
    }
}
