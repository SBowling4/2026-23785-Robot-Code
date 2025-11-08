package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Artifact;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Optional;

public class VisionSubsystem {
    private Limelight3A limelight;
    private LLResult result;
    private LLResultTypes.FiducialResult goodTag;

    private final HardwareMap hardwareMap;

    public boolean llValid = true;

    private static VisionSubsystem instance;

    public VisionSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);

        limelight.pipelineSwitch(0);

        limelight.start();
    }

    public void loop() {
        if (limelight == null) {
            llValid = false;
            return;
        }

        llValid = true;

        int goodTagId;

        if (Robot.alliance == Alliance.BLUE) {
            goodTagId = 20;
        } else if (Robot.alliance == Alliance.RED) {
            goodTagId = 19;
        } else {
            goodTagId = -1;
        }

        result = limelight.getLatestResult();

        if (result == null) return;

        for (LLResultTypes.FiducialResult fidResult : result.getFiducialResults()) {


            if (fidResult.getFiducialId() == goodTagId) {
                goodTag = fidResult;
            }

            if (VisionConstants.OBELISK_TAGS.contains(fidResult.getFiducialId())) {
                setMotif(fidResult.getFiducialId());
            }

        }

    }

    public Optional<Double> getTx() {
        if (result == null) return Optional.of(-1.0);
        if (goodTag == null) return Optional.of(-2.0);

        return Optional.of(goodTag.getTargetXDegrees());
    }

    public Optional<Double> getTy() {
        if (result == null) return Optional.of(-1.0);
        if (goodTag == null) return Optional.of(-2.0);

        return Optional.of(goodTag.getTargetYDegrees());
    }

    public Optional<Double> getTa() {
        if (result == null) return Optional.of(-1.0);
        if (goodTag == null) return Optional.of(-2.0);

        return Optional.of(goodTag.getTargetArea());
    }

    public double getDistance() {
        double scale = 3.2;
        Optional<Double> Ta = getTa();

        if (Ta.isEmpty()) return -1;

        return Math.sqrt(Ta.get() / scale);
    }

    public Optional<Double> getHorizontalAngle() {
        if (result == null || goodTag == null) return Optional.empty();

        Pose3D tagPose = goodTag.getTargetPoseRobotSpace();

        double x = tagPose.getPosition().x;
        double y = tagPose.getPosition().y;

        double horizontalAngleDegrees = Math.toDegrees(Math.atan2(y, x));

        return Optional.of(horizontalAngleDegrees);
    }

    public Optional<Double> getVerticalAngle() {
        if (result == null || goodTag == null) return Optional.empty();

        Pose3D tagPose = goodTag.getTargetPoseCameraSpace();

        double x = tagPose.getPosition().x;
        double y = tagPose.getPosition().y;
        double z = tagPose.getPosition().z;

        double dist = Math.sqrt(x*x + y*y);

        double verticalAngleRadians = Math.toDegrees(Math.atan2(z, dist));

        return Optional.of(verticalAngleRadians);
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
