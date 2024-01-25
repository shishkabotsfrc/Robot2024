package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Pigeon extends SubsystemBase {
  private Pigeon2 m_imu = new Pigeon2(DriveConstants.kPigeonCanId);
  private Rotation3d rotOffset = new Rotation3d();

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public void resetOffsets() {
    rotOffset = rotation();
  }

  public void setYaw(double r) {
    m_imu.setYaw(r);
  }

  public Rotation3d rotation() {
    return new Rotation3d(
        m_imu.getRoll().getValue(), m_imu.getPitch().getValue(), m_imu.getYaw().getValue());
  }

  public Rotation3d rotationCorrected() {
    return rotation().minus(rotOffset);
  }

  public double getAngle() {
    return rotation().getZ();
  }
}
