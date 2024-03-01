// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.commands.drive.PIDTuneCommand;
import frc.robot.commands.drive.ResetGyroOffsets;
import frc.robot.commands.drive.SetPose;
import frc.robot.commands.drive.XPositionLock;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.commands.rings.ShootAmp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import java.util.List;

public class RobotContainer {
  private final Limelight m_limelight = new Limelight("limelight");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight);
  private final Climber m_robotClimber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private Intake m_intake = new Intake();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, m_driverController));
  }

  private void configureButtonBindings() {
    // TODO: Use reasonable controls
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new XPositionLock(m_robotDrive));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new PIDTuneCommand(m_robotDrive));

    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(new SetPose(m_robotDrive, new Pose2d(0, 0, new Rotation2d(0, 0))));

    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new ResetGyroOffsets(m_robotDrive));

    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(
            new AlignShotCommand(m_robotDrive, m_shooter, m_intake, List.of(MarkerType.Amplifier)));

    new JoystickButton(m_driverController, Button.kB.value).onTrue(new FeedIntake(m_intake));

    new JoystickButton(m_driverController, Button.kBack.value).onTrue(new ShootAmp(m_intake));

    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new DrivetoSwerve(m_robotDrive, new Pose2d(1.5, 0, new Rotation2d(0.0))));
  }

  public Command getAutonomousCommand() {
    AlignShotCommand align =
        new AlignShotCommand(m_robotDrive, m_shooter, m_intake, List.of(MarkerType.Amplifier));
    DrivetoSwerve driveToSwerve =
        new DrivetoSwerve(
            m_robotDrive,
            new Pose2d(
                m_robotDrive.getPose().getX() - 2,
                m_robotDrive.getPose().getY(),
                new Rotation2d()));
    SequentialCommandGroup seq = new SequentialCommandGroup(align, driveToSwerve);
    return seq;
  }
}
