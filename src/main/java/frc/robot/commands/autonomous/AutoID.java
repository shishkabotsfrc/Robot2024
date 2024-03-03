package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class AutoID extends Command {
  private int ID;
  private DriveSubsystem m_robotDrive;
  private Intake m_intake;
  private Shooter m_shooter;

  public AutoID(int ID, DriveSubsystem drive, Intake intake, Shooter shooter) {
    this.ID = ID;
    this.m_robotDrive = drive;
    this.m_intake = intake;
    this.m_shooter = shooter;
    // TODO: change get1 --> into constructor that has if statements? find better solution
  }

  public SequentialCommandGroup get1(int position) {
    // Wait our turn
    Command wait = new WaitCommand(2);
    AlignShotCommand shoot =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    double blueSign = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;
    double sideStrafe = -2;
    // Instead of moving straight to the position, we move to the side
    // We dont want to run into our teammates
    DrivetoSwerve driveToSide =
        new DrivetoSwerve(
            m_robotDrive,
            new Pose2d(
                m_robotDrive.getPose().getX() + sideStrafe,
                m_robotDrive.getPose().getY(),
                new Rotation2d()));
    // Leave the danger zone
    // TODO: Check if this will move back or it will slam into the wall
    DrivetoSwerve driveToBack =
        new DrivetoSwerve(
            m_robotDrive,
            new Pose2d(
                m_robotDrive.getPose().getX() + sideStrafe,
                m_robotDrive.getPose().getY() + blueSign * 3,
                new Rotation2d()));
    return new SequentialCommandGroup(wait, shoot, driveToSide, driveToBack);
  }

  public SequentialCommandGroup get2(int position) {
    double[][] nextPos = new double[2][3];
    AlignShotCommand shootPre =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    autoStartLine driveRight = new autoStartLine(m_robotDrive, m_intake, m_shooter, 1, nextPos);
    autoLine check =
        new autoLine(
            m_robotDrive,
            m_intake,
            m_shooter,
            m_robotDrive.getPose().getX(),
            m_robotDrive.getPose().getY(),
            1,
            nextPos);

    return new SequentialCommandGroup(shootPre, driveRight, check);
  }
}
