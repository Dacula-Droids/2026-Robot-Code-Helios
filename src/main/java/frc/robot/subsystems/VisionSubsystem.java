package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.FieldConstants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem INSTANCE = new VisionSubsystem();

  public static VisionSubsystem getInstance() {
    return INSTANCE;
  }

  public static SwerveDrive swerveDrive = SwerveSubsystem.getInstance().swerveDrive;

  private static final String FRONT_LIMELIGHT = "limelight-front";
  private static final String BACK_LIMELIGHT = "limelight-back";

  private final String[] limelights = { FRONT_LIMELIGHT, BACK_LIMELIGHT };

  public VisionSubsystem() {
  }

  // Calculates the standard deviations based on tag count and distance.

  public Matrix<N3, N1> calculateStdDevs(String limelightName, Pose2d estimatedPose) {
    var result = LimelightHelpers.getLatestResults(limelightName);

    // If no targets or bad data, return single-tag default
    if (!LimelightHelpers.getTV(limelightName) || result == null || result.targets_Fiducials.length == 0
        || estimatedPose == null) {
      return VisionConstants.kSingleTagStdDevs;
    }

    int numTags = 0;
    double totalDist = 0.0;

    for (var fid : result.targets_Fiducials) {
      // Find where this tag actually is on the field
      Optional<Pose3d> tagPose = FieldConstants.aprilTagFieldLayout.getTagPose((int) fid.fiducialID);

      if (tagPose.isPresent()) {
        numTags++;
        // Calculate distance from Robot to the Tag
        totalDist += tagPose.get().toPose2d().getTranslation()
            .getDistance(estimatedPose.getTranslation());
      }
    }

    double avgTagDistance = numTags > 0 ? totalDist / numTags : 0.0;

    if (numTags == 0) {
      return VisionConstants.kSingleTagStdDevs;
    } else if (numTags > 1) {
      return VisionConstants.kMultiTagStdDevs;
    } else {
      // Scale standard deviation up quadratically as the robot gets further from a
      // single tag
      return VisionConstants.kSingleTagStdDevs.times(1 + (avgTagDistance * avgTagDistance / 30.0));
    }
  }

  /**
   * Gets the accurate timestamp of the pose by subtracting network and capture
   * latency.
   */
  public double getTimestamp(String limelightName) {
    return Timer.getFPGATimestamp()
        - (LimelightHelpers.getLatency_Pipeline(limelightName) / 1000.0)
        - (LimelightHelpers.getLatency_Capture(limelightName) / 1000.0);
  }

  @Override
  public void periodic() {
    // Get the robots current heading from the Swerve Drive (For MegaTag2)
    Rotation2d robotYaw = swerveDrive.getYaw();

    double robotPitch = swerveDrive.getPitch().getDegrees();
    double robotRoll = swerveDrive.getRoll().getDegrees();

    // Process both Limelights, one after the other
    for (String ll : limelights) {

      // Seed the Limelight with the Gyro angle
      LimelightHelpers.SetRobotOrientation(ll, robotYaw.getDegrees(), 0.0, robotPitch, 0.0, robotRoll, 0.0);

      // Check if this specific camera sees a target
      if (LimelightHelpers.getTV(ll)) {

        // Grab the pose mapped to the WPILib Blue Origin
        Pose2d estimatedPose = LimelightHelpers.getBotPose2d_wpiBlue(ll);

        if (estimatedPose != null) {
          // Do the dynamic standard deviation math for THIS camera
          Matrix<N3, N1> stdDevs = calculateStdDevs(ll, estimatedPose);
          double timestamp = getTimestamp(ll);

          // Feed the pose to YAGSL / WPILib Pose Estimator
          swerveDrive.addVisionMeasurement(estimatedPose, timestamp, stdDevs);
        }
      }
    }
  }
}