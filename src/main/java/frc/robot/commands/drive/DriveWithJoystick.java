package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveWithJoystick extends Command {
  private final DriveSubsystem drive;
  private XboxController controller;

  public DriveWithJoystick(DriveSubsystem drive, XboxController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  public void execute() {
    drive.drive(
        -1 * -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband),
        -1 * -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband),
        -1 * -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband),
        true,
        true);
  }
}
