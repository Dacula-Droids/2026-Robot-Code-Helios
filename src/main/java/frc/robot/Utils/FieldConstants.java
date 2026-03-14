package frc.robot.Utils;

import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Utils.FieldTarget;
import frc.robot.Utils.OutpostTarget;

public class FieldConstants {

    // 1. STATIC LAYOUT
    public static final AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            // Load official 2026 Rebuilt layout
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load 2026 Field Layout", e);
        }
    }

    // 2. ID LOOKUP CLASS
    public static class AprilTagIDs {

        // April Tag IDs
        // Red Alliance
        public static final int RED_TRENCH_LEFT_FRONT_ID = 7;
        public static final int RED_TRENCH_LEFT_BACK_ID = 6;
        public static final int RED_TRENCH_RIGHT_FRONT_ID = 12;
        public static final int RED_TRENCH_RIGHT_BACK_ID = 1;
        public static final int RED_CLIMB_LEFT_ID = 16;
        public static final int RED_CLIMB_RIGHT_ID = 15;
        public static final int RED_OUTPOST_LEFT_ID = 14;
        public static final int RED_OUTPOST_RIGHT_ID = 13;

        // Blue Alliance
        public static final int BLUE_OUTPOST_LEFT_ID = 30;
        public static final int BLUE_OUTPOST_RIGHT_ID = 29;
        public static final int BLUE_TRENCH_LEFT_FRONT_ID = 23;
        public static final int BLUE_TRENCH_LEFT_BACK_ID = 22;
        public static final int BLUE_TRENCH_RIGHT_FRONT_ID = 28;
        public static final int BLUE_TRENCH_RIGHT_BACK_ID = 17;
        public static final int BLUE_CLIMB_LEFT_ID = 32;
        public static final int BLUE_CLIMB_RIGHT_ID = 31;
        public static final int BLUE_HUB_ID = 26;
        public static final int RED_HUB_ID = 10;


        public static int getAllianceOutpostLeftId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_OUTPOST_LEFT_ID;
            }
            return BLUE_OUTPOST_LEFT_ID;
        }

        public static int getAllianceOutpostRightId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_OUTPOST_RIGHT_ID;
            }
            return BLUE_OUTPOST_RIGHT_ID;
        }

        public static int getAllianceTrenchLeftFrontId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_TRENCH_LEFT_FRONT_ID;
            }
            return BLUE_TRENCH_LEFT_FRONT_ID;
        }

        public static int getAllianceTrenchLeftBackId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_TRENCH_LEFT_BACK_ID;
            }
            return BLUE_TRENCH_LEFT_BACK_ID;
        }

        public static int getAllianceTrenchRightFrontId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_TRENCH_RIGHT_FRONT_ID;
            }
            return BLUE_TRENCH_RIGHT_FRONT_ID;
        }

        public static int getAllianceTrenchRightBackId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_TRENCH_RIGHT_BACK_ID;
            }
            return BLUE_TRENCH_RIGHT_BACK_ID;
        }

        public static int getAllianceClimbLeftId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_CLIMB_LEFT_ID;
            }
            return BLUE_CLIMB_LEFT_ID;
        }

        public static int getAllianceClimbRightId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_CLIMB_RIGHT_ID;
            }
            return BLUE_CLIMB_RIGHT_ID;
        }

        public static int getAllianceHubId() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return RED_HUB_ID;
            }
            return BLUE_HUB_ID;
        }

    }

    // 3. FIELD DIMENSIONS (2026 Rebuilt)
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);

    public static class Outpost {
        public static final double wallWidth = Units.inchesToMeters(31.8);
        public static final double intakeHeight = Units.inchesToMeters(7.0);
        public static final double depthFromWall = Units.inchesToMeters(12.0);
    }

    // 4. THE OFFSET CALCULATOR
    public static Transform3d getFieldTargetOffset(FieldTarget target, boolean isOffset) {

        // --- OUTPOST OFFSETS ---
        if (target instanceof OutpostTarget) {
            // TUNE 0.45 BASED ON YOUR ROBOT'S BUMPER LENGTH!
            double distance = isOffset ? 0.45 : 1.0;
            return new Transform3d(
                    new Translation3d(distance, 0, 0),
                    new Rotation3d(0, 0, Math.PI) // Rotate 180 to face the tag
            );
        }

        // --- TRENCH OFFSETS ---
        else if (target instanceof TrenchTarget) {
            TrenchTarget trenchTarget = (TrenchTarget) target;
            double distance = isOffset ? 0.5 : 1.2;
            double rotationAngle;

            if (trenchTarget == TrenchTarget.LeftFront || trenchTarget == TrenchTarget.RightFront) {
                // Front targets: Face one direction (e.g., 180 degrees)
                rotationAngle = Math.PI;
            } else {
                // Back targets: Face the EXACT OPPOSITE direction (0 degrees)
                rotationAngle = 0.0;
            }

            return new Transform3d(
                    new Translation3d(distance, 0, 0),
                    new Rotation3d(0, 0, rotationAngle));
        }

        else if (target instanceof ClimbTarget) {
            double distance = isOffset ? 0.5 : 1.0;
            return new Transform3d(
                    new Translation3d(distance, 0, 0),
                    new Rotation3d(0, 0, Math.PI) // Rotate 180 degrees to face the tag
            );
        }

        else if (target instanceof HubTarget) {
            double distance = isOffset ? Units.inchesToMeters(60) : 1.0; //89 Inches
            return new Transform3d(
                    new Translation3d(distance, 0, 0),
                    new Rotation3d(0, 0, Math.PI) // Rotate 180 degrees to face the tag
            );
        }

        // Return empty transform if unknown target
        return new Transform3d();
    }
}