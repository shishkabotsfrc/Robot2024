// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drive.AlignShotCommand;
import frc.robot.commands.drive.DriveWithJoystick;
import frc.robot.commands.drive.DrivetoPose;
import frc.robot.commands.drive.PIDTuneCommand;
import frc.robot.commands.drive.ResetGyroOffsets;
import frc.robot.commands.drive.XPositionLock;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  private final Limelight m_limelight = new Limelight("limelight");
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new DriveWithJoystick(m_robotDrive, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO: Use reasonable controls
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new XPositionLock(m_robotDrive));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new PIDTuneCommand(m_robotDrive));
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new ResetGyroOffsets(m_robotDrive));
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new AlignShotCommand(m_robotDrive, m_limelight));
    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(
            new DrivetoPose(
                m_robotDrive,
                m_limelight,
                m_robotDrive.getPose().getX() + 0,
                m_robotDrive.getPose().getY() + 2));

    /*  new JoystickButton(m_driverController, Button.kY.value)
    .onTrue(new DrivetoPose(m_robotDrive, 10, 10));*/
  }

  // NOTE: // SwerveDrivePoseEstimator

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("===== AUTO =====");
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 0), new Translation2d(2, 0)
                // new Translation2d(1,-1)
                //  new Translation2d(0,-.5)
                ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, -1, new Rotation2d(0)),
            config);

    for (var t : exampleTrajectory.getStates()) {
      System.out.println(t.toString());
    }
    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
