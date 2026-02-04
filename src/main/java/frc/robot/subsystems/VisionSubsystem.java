package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem INSTANCE = new VisionSubsystem();

  public static VisionSubsystem getInstance() {
    return INSTANCE;
  }

  public static SwerveDrive swerveDrive = SwerveSubsystem.getInstance().swerveDrive;

  private static final String LIMELIGHT_NAME = "limelight";
  private int tagCount = 0;
  private double avgTagDistance = 0.0;
  private Matrix<N3, N1> curStdDevs = VisionConstants.kSingleTagStdDevs;
  private Pose2d estimatedPose;

  public VisionSubsystem() {
  }

  public int getTagCount() {
    return tagCount;
  }

  public double getAvgTagDistance() {
    return avgTagDistance;
  }

  public Matrix<N3, N1> getVisionStdDevs() {
    return curStdDevs;
  }

  public void updateStdDevs() {
    var result = LimelightHelpers.getLatestResults(LIMELIGHT_NAME);
    if (!hasValidTarget() || result == null || result.targets_Fiducials.length == 0 || estimatedPose == null) {
      tagCount = 0;
      avgTagDistance = 0.0;
      curStdDevs = VisionConstants.kSingleTagStdDevs;
      return;
    }

    int numTags = 0;
    double totalDist = 0.0;

    for (var fid : result.targets_Fiducials) {
      // You only have fiducial info here; to get distance, you need a map of tag
      // positions.
      numTags++;
      // Example: totalDist += distanceToTag(fid.id); <-- requires field tag map
    }

    tagCount = numTags;
    avgTagDistance = numTags > 0 ? totalDist / numTags : 0.0;

    if (numTags == 0) {
      curStdDevs = VisionConstants.kSingleTagStdDevs;
    } else if (numTags > 1) {
      curStdDevs = VisionConstants.kMultiTagStdDevs;
    } else {
      curStdDevs = VisionConstants.kSingleTagStdDevs.times(1 + (avgTagDistance * avgTagDistance / 30.0));
    }
  }

  public Matrix<N3, N1> getCurrentStdDevs() {
    updateVisionMeasurements();
    return curStdDevs;
  }

  public void updateVisionMeasurements() {
    estimatedPose = getEstimatedPose();
    updateStdDevs();
  }

  public boolean hasValidTarget() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  public Pose2d getEstimatedPose() {
    Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);
    if (pose != null)
      estimatedPose = pose;
    return estimatedPose;
  }

  public double getTimestamp() {
    return Timer.getFPGATimestamp()
        - (LimelightHelpers.getLatency_Pipeline(LIMELIGHT_NAME) / 1000.0)
        - (LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME) / 1000.0);
  }

  @Override
  public void periodic() {
    updateVisionMeasurements();

    if (hasValidTarget() && estimatedPose != null) {
      swerveDrive.addVisionMeasurement(
          estimatedPose,
          getTimestamp(),
          curStdDevs);
    }
  }

}
