package frc.robot.field;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utils.ApproachMode;
import frc.robot.Utils.FieldTarget;
import frc.robot.Utils.OutpostTarget;

public class FieldConstants {

    // STATIC LAYOUT
    public static final AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            // Load official 2026 Rebuilt layout
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load 2026 Field Layout", e);
        }
    }

    // This method basically takes the pose of the target, lets say april tag, moves
    // the pose of the april tag to the "translation" offset given, and also
    // "rotates" the pose
    // The pose produced is what the robot uses to get to the target
    public static Transform3d getFieldTargetOffset(FieldTarget target, boolean closeApproach) {
        if (target instanceof OutpostTarget) {
            double distance = closeApproach ? ApproachMode.CloseOutpost.offsetValue : ApproachMode.FarOutpost.offsetValue; // meters
            // NEGATE DISTANCE IF INVERTED!!!!
            return new Transform3d(new Translation3d(-distance, 0, 0), new Rotation3d(0, 0, Math.PI));
        }
        return new Transform3d(); // No offset
    }

    // ID LOOKUP CLASS
    public static class AprilTagIDs {

        // --- OUTPOST (Fuel Station) IDs ---
        // Red Alliance
        public static final int RED_OUTPOST_LEFT_ID = 14;
        public static final int RED_OUTPOST_RIGHT_ID = 13;

        // Blue Alliance
        public static final int BLUE_OUTPOST_LEFT_ID = 30;
        public static final int BLUE_OUTPOST_RIGHT_ID = 29;

        /**
         * Returns ID 14 (Red) or 30 (Blue)
         */
        public static int getAllianceOutpostLeftId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_OUTPOST_LEFT_ID;
            }
            return BLUE_OUTPOST_LEFT_ID;
        }

        /**
         * 
         */
        public static int getAllianceOutpostRightId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_OUTPOST_RIGHT_ID;
            }
            return BLUE_OUTPOST_RIGHT_ID;
        }
    }

    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);

    public static class Outpost {
        public static final double wallWidth = Units.inchesToMeters(31.8);
        public static final double intakeHeight = Units.inchesToMeters(7.0);
        public static final double depthFromWall = Units.inchesToMeters(12.0);
    }
}