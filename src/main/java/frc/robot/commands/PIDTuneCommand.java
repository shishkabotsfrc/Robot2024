package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class PIDTuneCommand extends Command {
  private final DriveSubsystem drive;
  int counter = 0;
  double startTime = WPIUtilJNI.now() * 1e-6;

  public PIDTuneCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  public void execute() {
    SwerveModuleState[] swerveState = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      swerveState[i] = new SwerveModuleState(0.0, new Rotation2d(Math.PI / 2));
    }
    drive.setModuleStates(swerveState);

    if (((Math.abs(drive.m_frontLeft.getState().angle.getRadians() - Math.PI / 2))) < 0.1) {
      counter++;
    } else {
      counter = 0;
    }

    if (counter > 20) {
      System.out.println((WPIUtilJNI.now() * 1e-6) - startTime);
    }
  }
}
