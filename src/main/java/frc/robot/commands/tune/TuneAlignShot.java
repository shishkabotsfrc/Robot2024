package frc.robot.commands.tune;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.field.AprilTagInfo.MarkerType;
import frc.field.Field;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TuneAlignShot extends Command {
  private DriveSubsystem m_drive;
  private XboxController xbox;

  public TuneAlignShot(DriveSubsystem drive, XboxController xbox) {
    this.m_drive = drive;
    addRequirements(drive);
  }

  public void execute() {
    m_drive.drive(
        0.2 * -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
        0.2 * -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
        0.2 * -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
        true,
        true);

    var other =
        Field.getClosestObjectivePoseByType(
            m_drive.getPose(), DriverStation.getAlliance().get(), List.of(MarkerType.Amplifier));
    Logger.recordOutput("AlignAlign/Translate", m_drive.getPose().minus(other));
    Logger.recordOutput("AlignAlign/Drive", m_drive.getPose());
    Logger.recordOutput("AlignAlign/Objective", other);
  }
}
