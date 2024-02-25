package frc.robot.commands.rings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.field.AprilTagInfo;
import frc.field.AprilTagInfo.MarkerType;
import frc.field.Field;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;

/** Drives to the nearest almplifier and shoots */
public class AlignShotCommand extends SequentialCommandGroup {
  private Transform2d OFFSET = new Transform2d();

  public AlignShotCommand(DriveSubsystem drive, Shooter shooter, Intake intake) {
    AprilTagInfo closestTag =
        Field.getClosestTagByType(
            drive.getPose(),
            DriverStation.getAlliance().get(),
            Set.of(MarkerType.Amplifier, MarkerType.Amplifier));
    Pose2d closestTagPose = closestTag.pose().toPose2d();
    // TODO: Does this move offset forward from perspective of apriltag?

    Pose2d m_targetPose = closestTagPose.plus(OFFSET);
    addCommands(new DrivetoSwerve(drive, m_targetPose), new ShootCommand(shooter, intake));
  }
}
