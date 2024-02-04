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

  public Pigeon() {
    m_imu.setYaw(0.);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/Angle", getAngle());
    Logger.recordOutput(getName() + "/AccelX", getAccelerationX());
    Logger.recordOutput(getName() + "/AccelY", getAccelerationY());
    Logger.recordOutput(getName() + "/AccelZ", getAccelerationZ());
  }

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
        Units.degreesToRadians(m_imu.getRoll().refresh().getValueAsDouble()),
        Units.degreesToRadians(m_imu.getPitch().refresh().getValueAsDouble()),
        Units.degreesToRadians(m_imu.getYaw().refresh().getValueAsDouble()));
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
   * Returns the yaw of the robot.
   *
   * @return the robot's yaw in degrees, from -180 to 180
   */
  public double getAngle() {
    return m_imu.getYaw().getValueAsDouble();
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
}
