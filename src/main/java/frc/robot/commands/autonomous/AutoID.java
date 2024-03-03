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

  private DriveSubsystem m_robotDrive;
  private Intake m_intake;
  private Shooter m_shooter;
  private double[][] nextPos = new double[2][3];

  public AutoID(DriveSubsystem drive, Intake intake, Shooter shooter) {
    this.m_robotDrive = drive;
    this.m_intake = intake;
    this.m_shooter = shooter;
  }

  public SequentialCommandGroup get1() {
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

  public SequentialCommandGroup get2(int iter) {
    AlignShotCommand shootPre =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    autoStartLine driveRight =
        new autoStartLine(m_robotDrive, m_intake, m_shooter, iter - 1, nextPos);
    autoLine check =
        new autoLine(
            m_robotDrive,
            m_intake,
            m_shooter,
            m_robotDrive.getPose().getX(),
            m_robotDrive.getPose().getY(),
            iter,
            nextPos);

    return new SequentialCommandGroup(shootPre, driveRight, check);
  }

  public SequentialCommandGroup get3(int pos) {
    double startPosX = m_robotDrive.getPose().getX();
    double startPosY = m_robotDrive.getPose().getY();
    AlignShotCommand shootPre =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    autoStartLine getTheRing = new autoStartLine(m_robotDrive, m_intake, m_shooter, pos, nextPos);
    DrivetoSwerve driveForward =
        new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)));
    AlignShotCommand shootFinal =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    return new SequentialCommandGroup(shootPre, getTheRing, driveForward, shootFinal);
  }
}
