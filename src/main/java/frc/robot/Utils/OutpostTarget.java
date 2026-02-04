package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagIDs;

public enum OutpostTarget implements FieldTarget {

    Left(
            AprilTagIDs.getAllianceOutpostLeftId(),
            FieldConstants.aprilTagFieldLayout
                    .getTagPose(AprilTagIDs.getAllianceOutpostLeftId())
                    .orElseThrow()),

    Right(
            AprilTagIDs.getAllianceOutpostRightId(),
            FieldConstants.aprilTagFieldLayout
                    .getTagPose(AprilTagIDs.getAllianceOutpostRightId())
                    .orElseThrow());

    private final int aprilTagId;
    private final Pose3d targetPose;

    OutpostTarget(int aprilTagId, Pose3d targetPose) {
        this.aprilTagId = aprilTagId;
        this.targetPose = targetPose;
    }

    @Override
    public int getAprilTagId() {
        return aprilTagId;
    }

    @Override
    public Pose3d getTargetPose() {
        return targetPose;
    }
}
