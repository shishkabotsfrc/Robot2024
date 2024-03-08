// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.vision.Limelight;
import frc.utils.CurrentTime;
import frc.utils.SwerveUtils;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  public MAXSwerveModule[] modules = new MAXSwerveModule[4];
  public SwerveModuleState[] m_desiredStates = new SwerveModuleState[4];

  // The gyro sensor
  public final Pigeon m_imu = new Pigeon();
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private double m_inputTranslationDir = 0.0;
  private double m_inputTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = CurrentTime.seconds();

  public Limelight m_limelight = null;

  public SwerveDriveOdometry m_odometry = null;
  PoseEstimator m_estimator;

  public DriveSubsystem(Limelight limelight) {
    m_limelight = limelight;
    modules[ModuleId.FL.index()] =
        new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    modules[ModuleId.FR.index()] =
        new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    modules[ModuleId.RL.index()] =
        new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    modules[ModuleId.RR.index()] =
        new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    for (int i = 0; i < 4; i++) {
      // m_desiredStates[i] = modules[i].getState();
      m_desiredStates[i] = new SwerveModuleState();
    }

    m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(m_imu.getAngle()),
            new SwerveModulePosition[] {
              modules[ModuleId.FL.index()].getPosition(),
              modules[ModuleId.FR.index()].getPosition(),
              modules[ModuleId.RL.index()].getPosition(),
              modules[ModuleId.RR.index()].getPosition()
            });
    m_estimator = new PoseEstimator(this);

    AutoBuilder.configureRamsete(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::drive, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public void modulesExecute(Consumer<MAXSwerveModule> closure) {
    for (int i = 0; i < modules.length; i++) {
      closure.accept(modules[i]);
    }
  }

  @Override
  public void periodic() {
    // System.out.println("--- periodic step");

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_imu.getAngle()),
        new SwerveModulePosition[] {
          modules[ModuleId.FL.index()].getPosition(),
          modules[ModuleId.FR.index()].getPosition(),
          modules[ModuleId.RL.index()].getPosition(),
          modules[ModuleId.RR.index()].getPosition()
        });
    logState(getName());
  }

  public SwerveModulePosition[] getModuleStates() {
    return new SwerveModulePosition[] {
      modules[ModuleId.FL.index()].getPosition(),
      modules[ModuleId.FR.index()].getPosition(),
      modules[ModuleId.RL.index()].getPosition(),
      modules[ModuleId.RR.index()].getPosition()
    };
  }

  @Override
  public void simulationPeriodic() {}

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_estimator.getPose();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_imu.getAngle()),
        new SwerveModulePosition[] {
          modules[ModuleId.FL.index()].getPosition(),
          modules[ModuleId.FR.index()].getPosition(),
          modules[ModuleId.RL.index()].getPosition(),
          modules[ModuleId.RR.index()].getPosition()
        },
        pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        modules[ModuleId.FL.index()].getState(),
        modules[ModuleId.FR.index()].getState(),
        modules[ModuleId.RL.index()].getState(),
        modules[ModuleId.RR.index()].getState());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false,
        false);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    // Preserve wheel direction when stopped, stops robot from snapping at end
    if (Math.abs(xSpeed) < 0.01 && Math.abs(ySpeed) < 0.01 && Math.abs(rot) < 0.01) {
      modulesExecute(
          mod -> mod.setDesiredState(new SwerveModuleState(0, new Rotation2d(mod.getAngle()))));
      return;
    }
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      m_inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      m_inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = CurrentTime.seconds();
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(m_inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, m_inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(m_inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(m_inputTranslationMag);
        }
      } else {
        m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDir, m_inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    m_desiredStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(m_imu.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    modules[0].setDesiredState(m_desiredStates[0]);
    modules[1].setDesiredState(m_desiredStates[1]);
    modules[2].setDesiredState(m_desiredStates[2]);
    modules[3].setDesiredState(m_desiredStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setXWheels() {
    modules[ModuleId.FL.index()].setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    modules[ModuleId.FR.index()].setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[ModuleId.RL.index()].setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    modules[ModuleId.RR.index()].setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    modules[0].setDesiredState(desiredStates[0]);
    modules[1].setDesiredState(desiredStates[1]);
    modules[2].setDesiredState(desiredStates[2]);
    modules[3].setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    modulesExecute(MAXSwerveModule::resetEncoders);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_imu.setYaw(0.);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in raidans, from -pi/2 to pi/2
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_imu.getAngle()).getRadians();
  }

  /**
   * Logs the state of the system and its children.
   *
   * @param prefix The name of the module to log.
   */
  void logState(String prefix) {

    Logger.recordOutput(prefix + "/Input/Direction", m_inputTranslationDir);
    Logger.recordOutput(prefix + "/Input/Magnitude", m_inputTranslationMag);
    Logger.recordOutput(prefix + "/Pose2d(Odometry)", m_odometry.getPoseMeters());
    Logger.recordOutput(prefix + "/Pose2d(Estimator)", m_estimator.getPose());
    Logger.recordOutput(prefix + "/Heading", getHeading());

    Pose2d limelightPose = m_limelight.getPose(m_odometry.getPoseMeters().getRotation());

    Logger.recordOutput(prefix + "/Pose2dValid", limelightPose != null);
    if (limelightPose != null) {
      Logger.recordOutput(prefix + "/Pose2dLimelight", limelightPose);
    }

    Logger.recordOutput(prefix + "/DesiredSwerve", m_desiredStates);
    SwerveModuleState actual[] = new SwerveModuleState[4];
    actual[0] = modules[0].getState();
    actual[1] = modules[1].getState();
    actual[2] = modules[2].getState();
    actual[3] = modules[3].getState();
    Logger.recordOutput(prefix + "/ActuaSwerve", actual);

    modules[ModuleId.FL.index()].logState(prefix + "/Modules/FL");
    modules[ModuleId.FR.index()].logState(prefix + "/Modules/FR");
    modules[ModuleId.RL.index()].logState(prefix + "/Modules/RL");
    modules[ModuleId.RR.index()].logState(prefix + "/Modules/RR");
  }
}
