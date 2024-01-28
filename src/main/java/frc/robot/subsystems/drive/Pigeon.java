package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Pigeon extends SubsystemBase {
  private Pigeon2 m_imu = new Pigeon2(DriveConstants.kPigeonCanId);
  private Rotation3d rotOffset = new Rotation3d();

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  /** Reset the angular offsets to the */
  public void resetOffsets() {
    rotOffset = rotation();
  }

  /**
   * Set the yaw
   *
   * @param the yaw to set the IMU to.
   */
  public void setYaw(double r) {
    m_imu.setYaw(r);
  }

  /**
   * Return the 3 angles
   *
   * @returns three rotation angles
   */
  public Rotation3d rotation() {
    return new Rotation3d(
        m_imu.getRoll().getValue(), m_imu.getPitch().getValue(), m_imu.getYaw().getValue());
  }

  /**
   * Return the 3 angle rotation corrected by the offsets
   *
   * @returns three rotation angles (corrected)
   */
  public Rotation3d rotationCorrected() {
    return rotation().minus(rotOffset);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getAngle() {
    return rotation().getZ();
  }

  /** x acceleration */
  public double getAccelerationX() {
    return m_imu.getAccelerationX().getValueAsDouble();
  }

  /** y acceleration */
  public double getAccelerationY() {
    return m_imu.getAccelerationY().getValueAsDouble();
  }

  /** z acceleration */
  public double getAccelerationZ() {
    return m_imu.getAccelerationZ().getValueAsDouble();
  }

  /**
   * Logs the state of the system and its children.
   *
   * @param prefix The name of the module to log.
   */
  void logState(String prefix) {
    Logger.recordOutput(prefix + "/Angle", getAngle());
    Logger.recordOutput(prefix + "/AngleDegrees", Units.radiansToDegrees(getAngle()));
    Logger.recordOutput(prefix + "/AccelX", getAccelerationX());
    Logger.recordOutput(prefix + "/AccelY", getAccelerationY());
    Logger.recordOutput(prefix + "/AccelZ", getAccelerationZ());
    Logger.recordOutput(prefix + "/Rotation3d", rotation());
  }
}
