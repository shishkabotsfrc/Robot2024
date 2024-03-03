package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class AutoShootSeq extends SequentialCommandGroup {

  Integer iter;

  public AutoShootSeq(
      int moveX,
      int moveY,
      DriveSubsystem m_robotDrive,
      Intake m_intake,
      Shooter m_shooter,
      int _iter) {
    iter = _iter; // THIS IS NECESSARY, DONT TOUCH
    double startPosX = m_robotDrive.getPose().getX();
    double startPosY = m_robotDrive.getPose().getY();

    addCommands(new DrivetoSwerve(m_robotDrive, new Pose2d(moveX, moveY, new Rotation2d(0.0))), new FeedIntake(m_intake));

    /*if (iter == 1) {
      addCommands(
          new checkIntake(m_robotDrive, m_intake, m_shooter, startPosX, startPosY),
          new checkIntake(m_robotDrive, m_intake, m_shooter, startPosX, startPosY));
      new Trigger(() -> m_intake.getIntakeHasNote())
          .onTrue(
              new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                  .andThen(
                      new AlignShotCommand(
                          m_robotDrive,
                          m_shooter,
                          m_intake,
                          List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter))))
          .onFalse(null);
    }

    if (iter == 2) {
      addCommands(new checkIntake(m_robotDrive, m_intake, m_shooter, startPosX, startPosY));

      new Trigger(() -> m_intake.getIntakeHasNote())
          .onTrue(
              new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                  .andThen(
                      new AlignShotCommand(
                          m_robotDrive,
                          m_shooter,
                          m_intake,
                          List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter))))
          .onFalse(null);
    }

    if (iter == 3) {
      new Trigger(() -> m_intake.getIntakeHasNote())
          .onTrue(
              new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                  .andThen(
                      new AlignShotCommand(
                          m_robotDrive,
                          m_shooter,
                          m_intake,
                          List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter))))
          .onFalse(null);
    }
  */}
}
