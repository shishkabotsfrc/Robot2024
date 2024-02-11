package frc.robot.commands.tune;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.SwerveUtils;

public class AngleDriver {

  private static final double DELTA = 0.1;
  private DriveSubsystem m_drive;
  private double m_targetAngle;

  SwerveModuleState[] targetSwerveStates = new SwerveModuleState[4];

  public AngleDriver(DriveSubsystem drive, double targetAngle) {
    m_drive = drive;
    m_targetAngle = targetAngle;
  }

  private static boolean epsilonDelta(double v1, double v2, double delta) {
    return Math.abs(v2 - v1) < delta;
  }

  public double getTargetAngle() {
    return m_targetAngle;
  }

  public void setTargetAngle(double targetAngle) {
    m_targetAngle = targetAngle;
    for (int i = 0; i < 4; i++) {
      targetSwerveStates[i] = new SwerveModuleState(0.0, new Rotation2d(targetAngle));
    }
  }

  public boolean isAllAtTarget() {
    for (int i = 0; i < 4; i++) {
      double angle = getAngle(i);
      boolean arrived = epsilonDelta(m_targetAngle, angle, DELTA);
      if (!arrived) {
        return false;
      }
    }
    return true;
  }

  public double getAngle(int index) {
    return SwerveUtils.WrapAngle(m_drive.modules[index].getAngle());
  }

  public void drive() {
    m_drive.setModuleStates(targetSwerveStates);
  }
}
