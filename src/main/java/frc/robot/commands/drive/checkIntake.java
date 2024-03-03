package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class checkIntake {

  public checkIntake(
      DriveSubsystem m_robotDrive,
      Intake m_intake,
      Shooter m_shooter,
      double startPosX,
      double startPosY) {

    new Trigger(() -> m_intake.getIntakeHasNote())
        .onTrue(
            new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                .andThen(
                    new AlignShotCommand(
                        m_robotDrive,
                        m_shooter,
                        m_intake,
                        List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter))))
        .onFalse(
            new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                .andThen(new FeedIntake(m_intake)));
  }
}
