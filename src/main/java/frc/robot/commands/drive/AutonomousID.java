package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class AutonomousID extends Command {
  private int ID;
  private DriveSubsystem m_robotDrive;
  private Intake m_intake;
  private Shooter m_shooter;

  public AutonomousID(int ID, DriveSubsystem drive, Intake intake, Shooter shooter) {
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
    int xMoveRight, yMoveRight, xMoveMiddle, yMoveMiddle, xMoveLeft, yMoveLeft;

    // if(position == 2) {
    xMoveRight = 0;
    yMoveRight = 0;
    xMoveMiddle = 0;
    yMoveMiddle = 0;
    xMoveLeft = 0;
    yMoveLeft = 0;
    // }
    AlignShotCommand shootPre =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    // TODO: make driveBack/driveLeft not run if already covered in driveRight, or just combine
    // everything into autoshootseq
    AutoShootSeq driveRight =
        new AutoShootSeq(xMoveRight, yMoveRight, m_robotDrive, m_intake, m_shooter, 1);
    AutoShootSeq driveBack =
        new AutoShootSeq(xMoveMiddle, yMoveMiddle, m_robotDrive, m_intake, m_shooter, 2);
    AutoShootSeq driveLeft =
        new AutoShootSeq(xMoveLeft, yMoveLeft, m_robotDrive, m_intake, m_shooter, 3);

    return new SequentialCommandGroup(shootPre, driveRight, driveBack, driveLeft);
  }
}
