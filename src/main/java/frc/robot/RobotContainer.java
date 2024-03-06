// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.XPositionLock;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.commands.rings.ShootCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;

public class RobotContainer {
  private final Limelight m_limelight = new Limelight("limelight");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight);
  // private final Climber m_robotClimber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private Intake m_intake = new Intake();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, m_driverController));
  }

  private void configureButtonBindings() {
    // TODO: Use reasonable controls

    //  new JoystickButton(m_driverController, Button.kRightBumper.value)
    //    .onTrue(new PIDTuneCommand(m_robotDrive));

    // new JoystickButton(m_driverController, Button.kX.value)
    //  .onTrue(new SetPose(m_robotDrive, new Pose2d(0, 0, new Rotation2d(0, 0))));

    new JoystickButton(m_driverController, Button.kX.value).whileTrue(new ShootCommand(m_shooter));

    // stops the robot
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new XPositionLock(m_robotDrive));

    // shoots the ring
    /*new JoystickButton(m_driverController, Button.kRightBumper.value)
            .onTrue(
                new AlignShotCommand(m_robotDrive, m_shooter, m_intake, List.of(MarkerType.Amplifier)));
    */
    // feed intake

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new FeedIntake(m_intake));

    // // shoot the amp
    // new JoystickButton(m_driverController, Button.kA.value).onTrue(new ShootAmp(m_intake));

    // // climb the rope
    // new JoystickButton(m_driverController, Button.kB.value).onTrue(new Climb(m_robotClimber));

    // new JoystickButton(m_driverController, Button.kStart.value)
    //   .onTrue(new DrivetoSwerve(m_robotDrive, new Pose2d(1.5, 0, new Rotation2d(0.0))));
    // new JoystickButton(m_driverController, Button.kA.value).whileTrue(new detectColorCommand());
  }

  // TODO: Manually change these values after talking with the alliance
  // TODO: This was written by someone who does not know how to play the game
  public Command getAutonomousCommand() {
    // AutoID auto = new AutoID(m_robotDrive, m_intake, m_shooter);
    // 1 is just wait, shoot, move (no input (Maybe add later))
    // 2 is go through line of 3 and shoot while doing so, moving back and forth is needed (input:
    // pos)
    // 3 is shoot, intake, shoot (input: pos)
    // TODO: configure for other side
    // return auto.get1();
    return null;
  }
}
