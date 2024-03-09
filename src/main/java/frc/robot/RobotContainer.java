// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ForceEject;
import frc.robot.commands.autonomous.AutoID;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.XPositionLock;
import frc.robot.commands.rings.FeedIntake;
import frc.robot.commands.rings.PrimeShooter;
import frc.robot.commands.rings.ShootAmp;
import frc.robot.commands.rings.StopShooter;
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
    //  autoChooser = AutoBuilder.buildAutoChooser();

    SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_helperController = new XboxController(OIConstants.kHelperControllerPort);

    public RobotContainer() {
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        m_chooser.setDefaultOption(
                "BLUE Auto 1 Pos 1 is just wait, shoot, move (no input (Maybe add later))",
                new AutoID(m_robotDrive, m_intake, m_shooter, 1, 1, "blue"));
        m_chooser.addOption(
                "BLUE Auto 1 Pos 2 is just wait, shoot, move (no input (Maybe add later))",
                new AutoID(m_robotDrive, m_intake, m_shooter, 1, 2, "blue"));
        m_chooser.addOption(
                "BLUE Auto 1 Pos 3 is just wait, shoot, move (no input (Maybe add later))",
                new AutoID(m_robotDrive, m_intake, m_shooter, 1, 3, "blue"));

        m_chooser.addOption(
                "BLUE POS 1 Auto 2 is go through line of 3 and shoot while doing so, moving back and forth if needed",
                new AutoID(m_robotDrive, m_intake, m_shooter, 2, 1, "blue"));
        m_chooser.addOption("BLUE POS 2 Auto 2", new AutoID(m_robotDrive, m_intake, m_shooter, 2, 2, "blue"));
        m_chooser.addOption("BLUE POS 3 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 2, 3, "blue"));

        m_chooser.addOption(
                "BLUE POS 1 Auto 3 is shoot, intake, shoot",
                new AutoID(m_robotDrive, m_intake, m_shooter, 3, 1, "blue"));
        m_chooser.addOption("BLUE POS 2 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 3, 2, "blue"));
        m_chooser.addOption("BLUE POS 3 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 3, 3, "blue"));
        m_chooser.addOption(
                "RED Auto 1 is just wait, shoot, move (no input (Maybe add later))",
                new AutoID(m_robotDrive, m_intake, m_shooter, 1, 1, "red"));

        m_chooser.addOption(
                "RED POS 1 Auto 2 is go through line of 3 and shoot while doing so, moving back and forth if needed",
                new AutoID(m_robotDrive, m_intake, m_shooter, 2, 1, "red"));
        m_chooser.addOption("RED POS 2 Auto 2", new AutoID(m_robotDrive, m_intake, m_shooter, 2, 2, "red"));
        m_chooser.addOption("RED POS 3 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 2, 3, "red"));

        m_chooser.addOption(
                "RED POS 1 Auto 3 is shoot, intake, shoot",
                new AutoID(m_robotDrive, m_intake, m_shooter, 3, 1, "red"));
        m_chooser.addOption("RED POS 2 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 3, 2, "red"));
        m_chooser.addOption("RED POS 3 Auto 3", new AutoID(m_robotDrive, m_intake, m_shooter, 3, 3, "red"));

        SmartDashboard.putData(m_chooser);

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
        new JoystickButton(m_helperController, Button.kLeftBumper.value)
                .onTrue(new FeedIntake(m_intake));

        // shoots the ring + auto  adjust
        // new JoystickButton(m_helperController, Button.kRightStick.value)
        //     .onTrue(
        //         new AlignShotCommand(
        //             m_robotDrive, m_shooter, m_intake, List.of(MarkerType.SpeakerCenter)));

        // shoot the amp
        new JoystickButton(m_helperController, Button.kRightBumper.value)
                .onTrue(new ShootAmp(m_intake));

        new JoystickButton(m_helperController, Button.kRightStick.value).onTrue(new ShootAmp(m_intake));

        // extras
        new JoystickButton(m_helperController, Button.kA.value).onTrue(new StopShooter(m_shooter));
        new JoystickButton(m_helperController, Button.kStart.value).onTrue(new PrimeShooter(m_shooter));
        new JoystickButton(m_helperController, Button.kB.value).onTrue(new ForceEject(m_intake));

        // climb the rope
        // new JoystickButton(m_driverController, Button.kA.value).onTrue(new Climb(m_robotClimber));

        // new JoystickButton(m_driverController, Button.kStart.value)
        //   .onTrue(new DrivetoSwerve(m_robotDrive, new Pose2d(1.5, 0, new Rotation2d(0.0))));
        // new JoystickButton(m_driverController, Button.kA.value).whileTrue(new detectColorCommand());
    }

    // TODO: Manually change these values after talking with the alliance
    // TODO: This was written by someone who does not know how to play the game
    public Command getAutonomousCommand() {
        //  return autoChooser.getSelected();

        //AutoID auto = new AutoID(m_robotDrive, m_intake, m_shooter);
        // 1 is just wait, shoot, move (no input (Maybe add later))
        //2 is go through line of 3 and shoot while doing so, moving back and forth is needed (input:
        // pos)
        //  3 is shoot, intake, shoot (input: pos)
        //  TODO: configure for other side
        return m_chooser.getSelected();
        // int sign = DriverStation.getAlliance().equals(Optional.of(Alliance.Blue)) ? 1 : -1;
//    SequentialCommandGroup hi =
//        new SequentialCommandGroup(
//            new PrimeShooter(m_shooter),
//            new WaitCommand(2),
//            new ForceEject(m_intake),
//            new DrivetoSwerve(
//                m_robotDrive,
//                new Pose2d(
//                    m_robotDrive.getPose().getX(),
//                    m_robotDrive.getPose().getY() - 2 /* *sign */,
//                    new Rotation2d(0.0))));
//    return hi;
    }
}
