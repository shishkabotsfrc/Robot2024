package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static NetworkTable netTable;

  public enum ledMode {
    ON,
    OFF,
    BLINK;
  }

  public Limelight() {
    netTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void periodic() {
    System.out.println(
        netTable.getEntry("tx").getDouble(0.0) + " " + netTable.getEntry("ty").getDouble(0.0));
  }
}
