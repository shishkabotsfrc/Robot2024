package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Pigeon;

public class ResetGyroOffsets extends Command {
  private final Pigeon pigeon;

  public ResetGyroOffsets(DriveSubsystem drive) {
    this.pigeon = drive.m_imu;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    pigeon.resetOffsets();
  }
}
