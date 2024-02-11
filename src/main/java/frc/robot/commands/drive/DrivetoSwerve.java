package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/** Sample command to illustrate the sample use. */
public class DrivetoSwerve extends Command {
  int counter = 0;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;
  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
  private DriveSubsystem drive;
  private Pose2d targetPose;
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
          Constants.AutoConstants.kMaxSpeedMetersPerSecond);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
          Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);

  public DrivetoSwerve(DriveSubsystem drive, Pose2d targetPose) {
    System.out.println("DrivetoSwerve: constructor");
    this.targetPose = targetPose;
    this.drive = drive;
    addRequirements(drive);

    xController =
        new ProfiledPIDController(AutoConstants.kPXController, 0, 0, DEFAULT_XY_CONSTRAINTS);

    yController =
        new ProfiledPIDController(AutoConstants.kPYController, 0, 0, DEFAULT_XY_CONSTRAINTS);

    thetaController =
        new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, DEFAULT_OMEGA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
  }

  @Override
  public void initialize() {
    System.out.println("DrivetoSwerve: initialize");
    Pose2d currPose = drive.getPose();
    thetaController.reset(currPose.getRotation().getRadians());
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  @Override
  public void execute() {
    Pose2d currPose = drive.getPose();
    double xSpeed = xController.calculate(targetPose.getX());
    double ySpeed = yController.calculate(targetPose.getY());
    double omegaSpeed = thetaController.calculate(targetPose.getRotation().getRadians());

    if (xController.atGoal()) {
      xSpeed = 0;
      // System.out.println("x: here");
    }

    if (yController.atGoal()) {
      ySpeed = 0;
      // System.out.println("y: here");
    }
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
      // System.out.println("rot: here");
    }
    drive.drive(-xSpeed, -ySpeed, omegaSpeed, true, true);
    if (counter % 100 == 0) {
      System.out.println(drive.getPose().getX() + ", " + drive.getPose().getY());
    }
    counter++;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("DrivetoSwerve: end: interrupted: " + Boolean.toString(interrupted));
    drive.drive(0, 0, 0, true, false);
  }

  @Override
  public boolean isFinished() {
    // System.out.println("DrivetoSwerve: isFinished");
    return atGoal();
  }
}
