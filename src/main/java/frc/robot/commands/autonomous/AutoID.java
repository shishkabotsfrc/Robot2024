package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.commands.ForceEject;
import frc.robot.commands.StowIntake;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.commands.drive.SetPose;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.commands.rings.PrimeShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class AutoID extends SequentialCommandGroup {

  private DriveSubsystem m_robotDrive;
  private Intake m_intake;
  private Shooter m_shooter;
  private double[][] nextPosRed3Line = new double[2][3];
  private double[][] nextPosBlue3Line = new double[2][3];
  private double[][] nextPos3Line = new double[2][3];
  private int ringOffset;
  private int intakeOffset;
  private String alliance;

  // TODO: get measurements

  public AutoID(
      DriveSubsystem drive, Intake intake, Shooter shooter, int ID, int iter, String alliance) {
    this.alliance = alliance;
    this.m_robotDrive = drive;
    this.m_intake = intake;
    this.m_shooter = shooter;
    // ringOffset = 14;
    intakeOffset = 25;
    nextPosRed3Line = fillRedArray(nextPosRed3Line);
    nextPosBlue3Line = fillBlueArray(nextPosBlue3Line);

    if (alliance.equals("blue")) {
      nextPos3Line = nextPosBlue3Line;
    } else {
      nextPos3Line = nextPosRed3Line;
    }
    if (ID == 1) {
      addCommands(get1(iter));
    } else if (ID == 2) {
      addCommands(get2(iter));
    } else {
      addCommands(get3(iter));
    }
  }

  private static double inchToMeter(double inch) {
    return inch * 0.0254;
  }

  private double[][] fillRedArray(double[][] arr) {
    for (int i = 0; i < arr[0].length; i++) {
      arr[1][i] = inchToMeter(538.73 + intakeOffset);
      arr[0][i] = inchToMeter(161.5 + (57 * i));
    }
    return arr;
  }

  private double[][] fillBlueArray(double[][] arr) {
    for (int i = 0; i < arr[0].length; i++) {
      arr[1][i] = 6.98 + inchToMeter(intakeOffset);

      arr[0][i] = 2.91 + inchToMeter((57 * i));
    }
    return arr;
  }

  public SequentialCommandGroup get1(int pos) {

    // Wait our turn
    Command wait = new WaitCommand(2);
    AlignShotCommand shoot =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    double blueSign = alliance.equals("blue") ? 1 : -1;
    // double blueSign = 0;
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
    return new SequentialCommandGroup(setStarting(pos), wait, shoot, driveToSide, driveToBack);
  }

  public SequentialCommandGroup get2(int iter) {

    AlignShotCommand shootPre =
        new AlignShotCommand(
            m_robotDrive,
            m_shooter,
            m_intake,
            List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    autoStartLine driveRight =
        new autoStartLine(m_robotDrive, m_intake, m_shooter, iter - 1, nextPos3Line);
    autoLine check =
        new autoLine(
            m_robotDrive,
            m_intake,
            m_shooter,
            m_robotDrive.getPose().getX(),
            m_robotDrive.getPose().getY(),
            iter,
            nextPos3Line);
    double a;
    if (alliance.equals("blue")) {
      a = inchToMeter(413.44);
    } else {
      a = inchToMeter(239.29);
    }
    DrivetoSwerve getOut =
        new DrivetoSwerve(
            m_robotDrive, new Pose2d(a, m_robotDrive.getPose().getY(), new Rotation2d(0.0)));
    return new SequentialCommandGroup(setStarting(iter), shootPre, driveRight, check, getOut);
  }

  public SequentialCommandGroup get3(int pos) {

    double startPosX = m_robotDrive.getPose().getX();
    double startPosY = m_robotDrive.getPose().getY();
    // AlignShotCommand shootPre =
    //     new AlignShotCommand(
    //         m_robotDrive,
    //         m_shooter,
    //         m_intake,
    //         List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
    PrimeShooter prep = new PrimeShooter(m_shooter);
    WaitCommand wait = new WaitCommand(2);
    ForceEject shootPre = new ForceEject(m_intake);

    autoStartLine getTheRing =
        new autoStartLine(m_robotDrive, m_intake, m_shooter, pos, nextPos3Line);
    FeedIntake feed = new FeedIntake(m_intake);
    StowIntake stow = new StowIntake(m_intake);
    DrivetoSwerve driveForward =
        new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)));
    ForceEject shootFinal = new ForceEject(m_intake);
    double a = inchToMeter(0);
    double b = inchToMeter(0);

    DrivetoSwerve getOut =
        new DrivetoSwerve(
            m_robotDrive,
            new Pose2d(
                m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY(), new Rotation2d(0.0)));

    if (alliance.equals("blue")) {

      if (pos == 1) {
        a = inchToMeter(139.76378);
        b = inchToMeter(88.1889764);
        getOut = new DrivetoSwerve(m_robotDrive, new Pose2d(a, b, new Rotation2d(0.0)));
      } else {
        a = inchToMeter(19.6850394);
        getOut =
            new DrivetoSwerve(
                m_robotDrive,
                new Pose2d(
                    m_robotDrive.getPose().getX() + a,
                    m_robotDrive.getPose().getY(),
                    new Rotation2d(0.0)));
      }
    } else {
      a = inchToMeter(239.29);
      if (pos == 1) {
        a = inchToMeter(501.181102);
        b = inchToMeter(88.1889764);
        getOut = new DrivetoSwerve(m_robotDrive, new Pose2d(a, b, new Rotation2d(0.0)));

      } else {
        a = inchToMeter(19.6850394);
        getOut =
            new DrivetoSwerve(
                m_robotDrive,
                new Pose2d(
                    m_robotDrive.getPose().getX() - a,
                    m_robotDrive.getPose().getY(),
                    new Rotation2d(0.0)));
      }
      getOut = new DrivetoSwerve(m_robotDrive, new Pose2d(a, a, new Rotation2d(0.0)));
    }
    return new SequentialCommandGroup(
        setStarting(pos),
        prep,
        wait,
        shootPre,
        getTheRing,
        feed,
        stow,
        driveForward,
        shootFinal,
        getOut);
  }

  /*public SequentialCommandGroup get4(int pos) {
      double startPosX = m_robotDrive.getPose().getX();
      double startPosY = m_robotDrive.getPose().getY();
      AlignShotCommand shootPre =
          new AlignShotCommand(
              m_robotDrive,
              m_shooter,
              m_intake,
              List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));
      autoStartLine getTheRing =
          new autoStartLine(m_robotDrive, m_intake, m_shooter, pos, nextPos3Line);
      DrivetoSwerve driveForward =
          new DrivetoSwerve(m_robotDrive, new Pose2d(startPosX, startPosY, new Rotation2d(0.0)));
      AlignShotCommand shootFinal =
          new AlignShotCommand(
              m_robotDrive,
              m_shooter,
              m_intake,
              List.of(MarkerType.SpeakerCenter, MarkerType.SpeakerOffCenter));


  double xRing, yRing, xWaypoint, yWaypoint;
  double middle = ??;

  DrivetoSwerve getWaypoint, getMiddle;
              if(pos==1) {
                xRing = middle;
                yRing = ??;

     getMiddle = new DrivetoSwerve(m_robotDrive, new Pose2d(xRing, yRing, new Rotation2d(0.0)));
              }
     if(pos==2) {
      xWaypoint =;
      yWaypoint = ;
      xRing = middle;
      yRing = ??;
      getWaypoint =new DrivetoSwerve(m_robotDrive, new Pose2d(xWaypoint, yWaypoint, new Rotation2d(0.0)));
      getMiddle = new DrivetoSwerve(m_robotDrive, new Pose2d(xRing, yRing, new Rotation2d(0.0)));

     }

     if(pos==3) {
      xWaypoint =;
      yWaypoint = ;
      xRing = middle;
      yRing = ??;
      getWaypoint =new DrivetoSwerve(m_robotDrive, new Pose2d(xWaypoint, yWaypoint, new Rotation2d(0.0)));
      getMiddle = new DrivetoSwerve(m_robotDrive, new Pose2d(xRing, yRing, new Rotation2d(0.0)));

     }







      double a;
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        a = inchToMeter(413.44);
      } else {
        a = inchToMeter(239.29);
      }
      DrivetoSwerve getOut =
          new DrivetoSwerve(
              m_robotDrive, new Pose2d(a, m_robotDrive.getPose().getY(), new Rotation2d(0.0)));
      return new SequentialCommandGroup(shootPre, getTheRing, driveForward, shootFinal, getOut);
    }*/

  public Command setStarting(int pos) {
    SetPose set = new SetPose(m_robotDrive, new Pose2d(0, 0, new Rotation2d(0)));

    if (pos == 3) {
      if (alliance.equals("blue")) {
        set = new SetPose(m_robotDrive, new Pose2d(0.76, 6.81, new Rotation2d(0)));
      } else {
        set = new SetPose(m_robotDrive, new Pose2d(15.06, 5.55, new Rotation2d(0)));
      }
    }
    if (pos == 2) {
      if (alliance.equals("blue")) {

      } else {

      }
    } else {
      if (alliance.equals("blue")) {

      } else {

      }
    }
    return set;
  }
}
