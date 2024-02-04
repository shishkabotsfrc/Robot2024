package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class XPositionLock extends Command {
  private final DriveSubsystem drive;

  public XPositionLock(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.setXWheels();
  }
}
