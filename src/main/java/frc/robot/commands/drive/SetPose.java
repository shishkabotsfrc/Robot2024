package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SetPose extends Command {

  Pose2d m_pose;
  DriveSubsystem m_driveSubsystem;

  public SetPose(DriveSubsystem driveSubsystem, Pose2d pose) {
    System.out.println("SET_POSE: constructor");
    m_driveSubsystem = driveSubsystem;
    m_pose = pose;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("SET_POSE: initialize");
  }

  @Override
  public void execute() {
    System.out.println("SET_POSE: execute");
    m_driveSubsystem.resetOdometry(m_pose);
  }

  @Override
  public boolean isFinished() {
    System.out.println("SET_POSE: isFinished");
    return true;
  }
}
