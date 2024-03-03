package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;

public class autoStartLine extends SequentialCommandGroup {

  Integer iter;

  public autoStartLine(
      DriveSubsystem m_robotDrive,
      Intake m_intake,
      Shooter m_shooter,
      int _iter,
      double[][] nextPos) {
    iter = _iter; // THIS IS NECESSARY, DONT TOUCH

    addCommands(
        new DrivetoSwerve(
            m_robotDrive, new Pose2d(nextPos[0][iter], nextPos[1][iter], new Rotation2d(0.0))),
        new FeedIntake(m_intake));
  }
}
