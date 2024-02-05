package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DrivetoPose extends Command {
  private final DriveSubsystem drive;
  private final Limelight vision;
  private Trajectory toPose;
  private ProfiledPIDController thetaController;
  double x, y, errorX, errorY;

  public DrivetoPose(DriveSubsystem drive, Limelight vision, double x, double y) {
    this.drive = drive;
    this.vision = vision;
    this.x = x;
    this.y = y;
    this.errorX = 0;
    this.errorY = 0;
    addRequirements(drive);
  }

  public void initalize() {
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    if (vision.getPose(new Rotation2d(0)) != null) {
      errorX += drive.getPose().getX() - vision.getPose(new Rotation2d(0)).getX();
      errorY += drive.getPose().getY() - vision.getPose(new Rotation2d(0)).getY();
    }

    Logger.recordOutput("Odometry/VisionErrorX", errorX);
    Logger.recordOutput("Odometry/VisionErrorY", errorY);

    if (errorX >= 0.25 || errorY >= 0.25) {

      System.out.println("error");
    }

    Pose2d robotPose = drive.getPose();
    Pose2d targetPose = new Pose2d(x, y, new Rotation2d(0));

    // All units in meters.
    System.out.println("===========================");
    System.out.println("POSE: " + robotPose.toString());
    System.out.println("TARGET: " + targetPose.toString());
    toPose =
        // TrajectoryGenerator.generateTrajectory(
        //     List.of(new Pose2d(x, y, new Rotation2d(0))), config);
        TrajectoryGenerator.generateTrajectory(
            robotPose,
            List.of(
                new Translation2d(
                    (robotPose.getX() + targetPose.getX()) / 2,
                    (robotPose.getY() + targetPose.getY()) / 2)),
            targetPose,
            config);

    thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void execute() {

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            toPose,
            drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(toPose.getInitialPose());
    swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0, false, false));
    System.out.println("done");
  }
}
