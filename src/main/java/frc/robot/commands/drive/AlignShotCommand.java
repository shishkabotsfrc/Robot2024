package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;

public class AlignShotCommand extends Command {
  private final DriveSubsystem drive;
  private final Limelight vision;

  public AlignShotCommand(DriveSubsystem drive, Limelight limelight) {
    this.drive = drive;
    this.vision = limelight;
    addRequirements(drive, limelight);
  }

  @Override
  public void execute() {
    if (!this.vision.mIO.validTarget) {
      this.drive.drive(0, 0, 0, false, false);
      return;
    }
    this.drive.drive(this.vision.mIO.ty * 5, -this.vision.mIO.tx * 5, 0, false, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
