// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.field.AprilTagInfo.MarkerType;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.detectColorCommand;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.DrivetoSwerve;
import frc.robot.commands.drive.XPositionLock;
import frc.robot.commands.rings.AlignShotCommand;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.commands.rings.ShootAmp;
import frc.robot.commands.rings.ShootCommand;
import frc.robot.commands.rings.StopShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import java.util.List;

public class RobotContainer {
  private final Limelight m_limelight = new Limelight("limelight");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight);
  // private final Climber m_robotClimber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private Intake m_intake = new Intake();
  //     autoChooser = AutoBuilder.buildAutoChooser();

  // SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_helperController = new XboxController(OIConstants.kHelperControllerPort);

  public RobotContainer() {
    // SmartDashboard.putData("Auto Chooser", autoChooser);

    // A chooser for autonomous commands
    // m_chooser.setDefaultOption(
    //     "Auto 1 is just wait, shoot, move (no input (Maybe add later))",
    //     new AutoID(m_robotDrive, m_intake, m_shooter, 1, 1));

    // m_chooser.addOption(
    //     "POS 1 Auto 2 is go through line of 3 and shoot while doing so, moving back and forth if
    // needed",
    //     new AutoID(m_robotDrive, m_intake, m_shooter, 2, 1));
    // m_chooser.addOption("POS 2 Auto 2", new AutoID(m_robotDrive, m_intake, m_shooter, 2, 2));
    // m_chooser.addOption("POS 3 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 2, 3));

    // m_chooser.addOption(
    //     "POS 1 Auto 3 is shoot, intake, shoot ",
    //     new AutoID(m_robotDrive, m_intake, m_shooter, 3, 1));
    // m_chooser.addOption("POS 2 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 3, 2));
    // m_chooser.addOption("POS 3 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 3, 3));

    // SmartDashboard.putData(m_chooser);

    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, m_driverController));
  }

  private void configureButtonBindings() {
    // TODO: Use reasonable controls

    //  new JoystickButton(m_driverController, Button.kRightBumper.value)
    //    .onTrue(new PIDTuneCommand(m_robotDrive));

    // new JoystickButton(m_driverController, Button.kX.value)
    //  .onTrue(new SetPose(m_robotDrive, new Pose2d(0, 0, new Rotation2d(0, 0))));

    // new JoystickButton(m_driverController, Button.kA.value)
    //     .whileTrue(new ShootCommand(m_shooter, m_intake));

    // stops the robot
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new XPositionLock(m_robotDrive));

    // feeds intake
    new JoystickButton(m_helperController, Button.kLeftStick.value)
        .onTrue(new FeedIntake(m_intake));

    // shoots the ring + auto  adjust
    new JoystickButton(m_helperController, Button.kRightStick.value)
        .onTrue(
            new AlignShotCommand(
                m_robotDrive, m_shooter, m_intake, List.of(MarkerType.SpeakerCenter)));

    // shoot the amp
    new JoystickButton(m_helperController, Button.kLeftBumper.value).onTrue(new ShootAmp(m_intake));

    // extras
    new JoystickButton(m_helperController, Button.kA.value).onTrue(new StopShooter(m_shooter));
    new JoystickButton(m_helperController, Button.kB.value).onTrue(new ShootCommand(m_shooter));
    new JoystickButton(m_helperController, Button.kY.value).whileTrue(new detectColorCommand());

    // climb the rope
    // new JoystickButton(m_driverController, Button.kA.value).onTrue(new Climb(m_robotClimber));

    // new JoystickButton(m_driverController, Button.kStart.value)
    //   .onTrue(new DrivetoSwerve(m_robotDrive, new Pose2d(1.5, 0, new Rotation2d(0.0))));
    // new JoystickButton(m_driverController, Button.kA.value).whileTrue(new detectColorCommand());
  }

  // TODO: Manually change these values after talking with the alliance
  // TODO: This was written by someone who does not know how to play the game
  public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();

    // AutoID auto = new AutoID(m_robotDrive, m_intake, m_shooter);
    // 1 is just wait, shoot, move (no input (Maybe add later))
    // 2 is go through line of 3 and shoot while doing so, moving back and forth is needed (input:
    // pos)
    // 3 is shoot, intake, shoot (input: pos)
    // TODO: configure for other side
    // return m_chooser.getSelected();
    return new DrivetoSwerve(m_robotDrive, new Pose2d(0, 0, new Rotation2d(0.0)));
  }
}
