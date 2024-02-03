package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.EnumSet;

public class Limelight extends SubsystemBase {
  private static NetworkTable netTable;
  private DetectedTarget target = new DetectedTarget();

  public PeriodicIO mIO = new PeriodicIO();

  public class PeriodicIO {
    boolean validTarget;
    public double tx, ty;
    double area;
    double latency;
    long inputLed;
    long targetID;
    int inputCamMode;
  }

  public enum LedMode {
    PIPELINE,
    OFF,
    BLINK,
    ON
  }

  public Limelight(String name) {
    netTable = NetworkTableInstance.getDefault().getTable(name);
    netTable.addListener("json", EnumSet.of(Kind.kValueAll), new Listener());
  }

  /** Calls `updatePosition()` aysnchronously when networktable changed * */
  private class Listener implements TableEventListener {
    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
      if (key.equals("json")) {
        updatePosition();
      }
    }
  }

  /**
   * When limelight sent data over networkTable, update target information Docs:
   * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
   */
  public void updatePosition() {
    mIO.validTarget = netTable.getEntry("tv").getInteger(0) == 1;
    mIO.tx = netTable.getEntry("tx").getDouble(0.0);
    mIO.ty = netTable.getEntry("ty").getDouble(0.0);
    mIO.area = netTable.getEntry("ta").getDouble(0.0);
    mIO.latency = netTable.getEntry("tl").getDouble(0.0) + netTable.getEntry("cl").getDouble(0.0);
    mIO.targetID = netTable.getEntry("tid").getInteger(-1);
    mIO.inputLed = netTable.getEntry("ledMode").getInteger(0);
    target.update(mIO);
  }
}
