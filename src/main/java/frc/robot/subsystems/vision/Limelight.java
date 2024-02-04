package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.EnumSet;

public class Limelight extends SubsystemBase {
  private static NetworkTable netTable;
  public DetectedTarget target = new DetectedTarget();
  public PeriodicIO mIO = new PeriodicIO();

  public class PeriodicIO {
    public boolean validTarget;
    public double tx, ty;
    double area;
    double latency;
    long targetID;
    long ledMode;
    long camMode;
    long pipeline;
    long sreamMode;
    long snapshot;
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

  /** Calls `updatePosition()` aysnchronously when limelight finishes by publishing "json" * */
  private class Listener implements TableEventListener {
    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
      updatePosition();
    }
  }

  /**
   * When limelight sent data over networkTable, update target information Docs:
   * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
   */
  public void updatePosition() {
    mIO.validTarget = netTable.getEntry("tv").getInteger(0) == 1;
    mIO.tx = Units.degreesToRadians(netTable.getEntry("tx").getDouble(0.0));
    mIO.ty = Units.degreesToRadians(netTable.getEntry("ty").getDouble(0.0));
    mIO.area = netTable.getEntry("ta").getDouble(0.0);
    mIO.latency = netTable.getEntry("tl").getDouble(0.0) + netTable.getEntry("cl").getDouble(0.0);
    mIO.targetID = netTable.getEntry("tid").getInteger(-1);
    mIO.ledMode = netTable.getEntry("ledMode").getInteger(-1);
    mIO.camMode = netTable.getEntry("camMode").getInteger(-1);
    mIO.pipeline = netTable.getEntry("pipeline").getInteger(-1);
    mIO.sreamMode = netTable.getEntry("stream").getInteger(-1);
    mIO.snapshot = netTable.getEntry("snapshot").getInteger(-1);
    target.update(mIO);
  }

  /** Change the led mode on the limelight * */
  public void setLed(LedMode mode) {
    if (mode.ordinal() != mIO.ledMode) {
      netTable.getEntry("ledMode").setInteger(mode.ordinal());
    }
  }

  /**
   * Changes the camera operating mode. 0 for vision processing. 1 for driver camera (Increases
   * exposure, disables vision processing)
   */
  public void setCamMode(long mode) {
    if (mode < 0 || mode > 1) System.err.println(mode + " is not a valid camera mode");
    if (mode != mIO.camMode) {
      netTable.getEntry("camMode").setInteger(mode);
    }
  }

  /** Changes the pipeline of the camera (0-9) */
  public void setPipeline(long mode) {
    if (mode < 0 || mode > 9) System.err.println(mode + " is not a valid pipeline");
    if (mode != mIO.pipeline) {
      netTable.getEntry("pipeline").setInteger(mode);
    }
  }

  /** Changes the stream of the camera (0-2) */
  public void setStream(long mode) {
    if (mode < 0 || mode > 2) System.err.println(mode + " is not a valid camera stream");
    if (mode != mIO.sreamMode) {
      netTable.getEntry("stream").setInteger(mode);
    }
  }

  /** Changes the snapshot mode (0-1) */
  public void setSnapshot(long mode) {
    if (mode < 0 || mode > 1) System.err.println(mode + " is not a valid snapshot mode");
    if (mode != mIO.snapshot) {
      netTable.getEntry("snapshot").setInteger(mode);
    }
  }
}
