package frc.robot.commands.rings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.field.AprilTagInfo.MarkerType;
import frc.field.Field;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

/** Drives to the nearest almplifier or speaker and shoots */
public class AlignShotCommand extends SequentialCommandGroup {

  public AlignShotCommand(DriveSubsystem drive, Shooter shooter, Intake intake) {
    Pose2d m_targetPose =
        Field.getClosestObjectivePoseByType(
            drive.getPose(),
            DriverStation.getAlliance().get(),
            List.of(MarkerType.Amplifier, MarkerType.SpeakerCenter));
    addCommands(new DrivetoSwerve(drive, m_targetPose), new ShootCommand(shooter, intake));
  }
}
