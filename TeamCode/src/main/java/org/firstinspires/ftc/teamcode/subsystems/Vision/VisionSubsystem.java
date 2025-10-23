package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.Optional;

public class VisionSubsystem {
    private final Limelight3A limelight;
    private LLResult result;
    private LLResultTypes.FiducialResult goodTag;

    private HardwareMap hardwareMap;

    private static VisionSubsystem instance;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
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

            if (VisionConstants.OBELISK_TAGS.contains(fidResult.getFiducialId())) {
                setMotif(fidResult.getFiducialId());
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

    public void setMotif(int tagId) {
        if (Robot.hasMotif.get()) return;

        if (tagId == 21) {
            Robot.motif.put(1, Artifact.GREEN);
            Robot.motif.put(2, Artifact.PURPLE);
            Robot.motif.put(3, Artifact.PURPLE);

            Robot.hasMotif.set(true);
        }

        if (tagId == 22) {
            Robot.motif.put(1, Artifact.PURPLE);
            Robot.motif.put(2, Artifact.GREEN);
            Robot.motif.put(3, Artifact.PURPLE);

            Robot.hasMotif.set(true);
        }

        if (tagId == 23) {
            Robot.motif.put(1, Artifact.PURPLE);
            Robot.motif.put(2, Artifact.PURPLE);
            Robot.motif.put(3, Artifact.GREEN);

            Robot.hasMotif.set(true);
        }
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
