package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;

public class autoLine extends SequentialCommandGroup {

  public autoLine(
      DriveSubsystem m_robotDrive,
      Intake m_intake,
      Shooter m_shooter,
      double startPosX,
      double startPosY,
      int iter,
      double nextPos[][]) {
    for (int i = 0; i < iter - 1; i++) {
      // TODO: fix values

      /*  new Trigger(() -> m_intake.getIntakeHasNote())
              .onTrue(
                  new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                      .andThen(
                          new AlignShotCommand(
                              m_robotDrive,
                              m_shooter,
                              m_intake,
                              List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter)))
                      .andThen(
                          new DrivetoSwerve(
                              m_robotDrive,
                              new Pose2d(nextPos[0][iter], nextPos[1][iter], new Rotation2d(0.0)))))
              .onFalse(
                  new DrivetoSwerve(
                          m_robotDrive,
                          new Pose2d(nextPos[0][iter], nextPos[1][iter], new Rotation2d(0.0)))
                      .andThen(new FeedIntake(m_intake)));
        }
        new Trigger(() -> m_intake.getIntakeHasNote())
            .onTrue(
                new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)))
                    .andThen(
                        new AlignShotCommand(
                            m_robotDrive,
                            m_shooter,
                            m_intake,
                            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter))));
      }*/
    }
  }
}
