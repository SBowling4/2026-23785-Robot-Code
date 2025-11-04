package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleop.Artemis;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Artifact;

import java.util.Optional;

public class VisionSubsystem {
    private Limelight3A limelight;
    private LLResult result;
    private LLResultTypes.FiducialResult goodTag;

    private final HardwareMap hardwareMap;

    private static VisionSubsystem instance;

    private Pose2d visionBotPose;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);

    }

    public void loop() {
        int goodTagId;

        if (Artemis.alliance == Alliance.BLUE) {
            goodTagId = 20;
        } else if (Artemis.alliance == Alliance.RED) {
            goodTagId = 19;
        } else {
            goodTagId = -1;
        }

        result = limelight.getLatestResult();

        LLResult poseResult = result;

        for (LLResultTypes.FiducialResult fidResult : result.getFiducialResults()) {

            if (fidResult.getFiducialId() == goodTagId) {
                goodTag = fidResult;
            }

            if (VisionConstants.OBELISK_TAGS.contains(fidResult.getFiducialId())) {
                poseResult.getFiducialResults().remove(fidResult);
                setMotif(fidResult.getFiducialId());
            } else {
                poseResult.getFiducialResults().add(fidResult);
            }

        }

        Pose3D measuredPose = poseResult.getBotpose();

        visionBotPose = new Pose2d(measuredPose.getPosition().x, measuredPose.getPosition().y, new Rotation2d(measuredPose.getOrientation().getYaw()));

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
        if (Artemis.hasMotif.get()) return;

        if (tagId == 21) {
            Artemis.motif.put(1, Artifact.GREEN);
            Artemis.motif.put(2, Artifact.PURPLE);
            Artemis.motif.put(3, Artifact.PURPLE);

            Artemis.hasMotif.set(true);
        }

        if (tagId == 22) {
            Artemis.motif.put(1, Artifact.PURPLE);
            Artemis.motif.put(2, Artifact.GREEN);
            Artemis.motif.put(3, Artifact.PURPLE);

            Artemis.hasMotif.set(true);
        }

        if (tagId == 23) {
            Artemis.motif.put(1, Artifact.PURPLE);
            Artemis.motif.put(2, Artifact.PURPLE);
            Artemis.motif.put(3, Artifact.GREEN);

            Artemis.hasMotif.set(true);
        }
    }

    public Pose2d getVisionBotPose() {
        return visionBotPose;
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
