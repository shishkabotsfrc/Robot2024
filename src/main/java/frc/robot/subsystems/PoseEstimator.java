package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import org.littletonrobotics.junction.Logger;

// See for an example of how other teams do this:
// https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java

public class PoseEstimator extends SubsystemBase {
  private SwerveDrivePoseEstimator m_swerveEstimator;
  private DriveSubsystem m_driveSubsystem;
  private Limelight m_limelight;
  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  public PoseEstimator(DriveSubsystem drive) {
    m_driveSubsystem = drive;
    m_limelight = drive.m_limelight;
    m_swerveEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            new Rotation2d(m_driveSubsystem.getHeading()),
            m_driveSubsystem.getModuleStates(),
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);
  }

  public Pose2d getPose() {
    return m_swerveEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d pose) {
    m_swerveEstimator.resetPosition(
        new Rotation2d(m_driveSubsystem.getHeading()), m_driveSubsystem.getModuleStates(), pose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  @Override
  public void periodic() {
    if (m_limelight.isValid()) {
      Pose2d visionMeasurement = m_limelight.getPose(new Rotation2d(m_driveSubsystem.getHeading()));
      double resultTimestamp = m_limelight.mIO.millisTimeRecorded;
      m_swerveEstimator.addVisionMeasurement(visionMeasurement, resultTimestamp);
    }
    m_swerveEstimator.update(
        new Rotation2d(m_driveSubsystem.getHeading()), m_driveSubsystem.getModuleStates());

    Logger.recordOutput("PoseEstimator/getPose", getPose());
    Logger.recordOutput("PoseEstimator/swervePose", getPose());
    Logger.recordOutput("PoseEstimator/cameraPose", getPose());
  }
}
