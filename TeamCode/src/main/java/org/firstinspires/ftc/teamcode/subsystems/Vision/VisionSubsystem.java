package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.Optional;

public class VisionSubsystem {

    static HardwareMap hardwareMap;
    private final Limelight3A limelight;
    private LLResult result;

    private final List<Integer> obeliskTags = List.of(21, 22, 23);
    private LLResultTypes.FiducialResult goodTag;

    private static VisionSubsystem instance;

    public VisionSubsystem(HardwareMap hardwareMap) {
        VisionSubsystem.hardwareMap = hardwareMap;
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
    }

    public void init() {}

    public void loop() {
        int goodTagId;

        if (Robot.alliance == Alliance.BLUE) {
            goodTagId = 20;
        } else if (Robot.alliance == Alliance.RED) {
            goodTagId = 19;
        } else {
            goodTagId = -1;
        }

        result = limelight.getLatestResult();

        for (LLResultTypes.FiducialResult fidResult : result.getFiducialResults()) {

            if (fidResult.getFiducialId() == goodTagId) {
                goodTag = fidResult;
            }

            if (obeliskTags.contains(fidResult.getFiducialId()) && !Robot.hasMotif.get()) {
                if (fidResult.getFiducialId() == 21) {
                    Robot.motif.put(1, Artifact.GREEN);
                    Robot.motif.put(2, Artifact.PURPLE);
                    Robot.motif.put(3, Artifact.PURPLE);

                    Robot.hasMotif.set(true);
                }

                if (fidResult.getFiducialId() == 22) {
                    Robot.motif.put(1, Artifact.PURPLE);
                    Robot.motif.put(2, Artifact.GREEN);
                    Robot.motif.put(3, Artifact.PURPLE);

                    Robot.hasMotif.set(true);
                }

                if (fidResult.getFiducialId() == 23) {
                    Robot.motif.put(1, Artifact.PURPLE);
                    Robot.motif.put(2, Artifact.PURPLE);
                    Robot.motif.put(3, Artifact.GREEN);

                    Robot.hasMotif.set(true);
                }

            }
        }

    }

    public Optional<Double> getXDegrees() {
        if (!result.isValid()) {
            return Optional.empty();
        }

        return Optional.of(goodTag.getTargetXDegrees());
    }

    public Optional<Double> getYDegrees() {
        if (!result.isValid()) return Optional.empty();

        return Optional.of(goodTag.getTargetYDegrees());
    }


    public static VisionSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new VisionSubsystem(hardwareMap);
        }
        return instance;
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Vision not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }



}
