package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.field.Field;
import frc.utils.CurrentTime;
import java.util.EnumSet;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {

  // Constants to be moved out later
  // Translation from the center of the robot to the camera (robot coordinates).
  Translation3d cameraOffset = new Translation3d(-0.23, 0.0, 0.67);

  private static NetworkTable netTable;
  public DetectedTarget target = new DetectedTarget();
  public PeriodicIO mIO = new PeriodicIO();
  private Pose2d pose;

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

    public double millisTimeRecorded;
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
    pose = null;
  }

  /** Calls `updatePosition()` aysnchronously when limelight finishes by publishing "json" * */
  private class Listener implements TableEventListener {
    @Override
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
      if (key.equals("json")) {
        updateData();
      }
    }
  }

  /**
   * When limelight sent data over networkTable, update target information Docs:
   * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
   */
  public synchronized void updateData() {
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
    mIO.millisTimeRecorded = CurrentTime.millis();
    // target.update(mIO);
    Logger.recordOutput("Vision/Valid", mIO.validTarget);
    Logger.recordOutput("Vision/tx", mIO.tx);
    Logger.recordOutput("Vision/ty", mIO.ty);
  }

  /** If the current vision state is able to give a good pose * */
  public boolean isValid() {
    if (!mIO.validTarget) {
      return false;
    }
    if (mIO.ty < 0.1 && mIO.ty > -0.1) {
      System.err.println("Tag is level to camera: " + mIO.targetID);
      return false;
    }
    if (mIO.targetID == -1) {
      return false;
    }
    AprilTag tag = Field.kAprilTagMap.get((int) mIO.targetID);
    if (tag == null) {
      System.err.println("Uknown tag: " + mIO.targetID);
      return false;
    }
    if (tag.ID != mIO.targetID) {
      System.err.println("Tag does not match: " + mIO.targetID);
      return false;
    }
    return true;
  }

  public synchronized Pose2d getPose(Rotation2d rotation) {
    if (!isValid()) return null;
    AprilTag tag = Field.kAprilTagMap.get((int) mIO.targetID);
    double yaw = rotation.getRadians();
    double heightDiff = tag.pose.getZ() - cameraOffset.getZ();
    double distance_2d = heightDiff / Math.tan(mIO.ty);
    double beta = yaw - mIO.tx;
    double x = Math.cos(beta) * distance_2d;
    double y = Math.sin(beta) * distance_2d;
    // Translation from the tag to the camera
    Translation2d tagToCamera = new Translation2d(-x, -y);

    Pose2d cameraPose =
        new Pose2d(tag.pose.toPose2d().getTranslation().plus(tagToCamera), new Rotation2d(yaw));
    // TODO translate the camera pose to the robot pose

    Translation2d offset = cameraOffset.toTranslation2d().rotateBy(rotation);
    pose = new Pose2d(cameraPose.getTranslation().minus(offset), new Rotation2d(yaw));

    Logger.recordOutput("Vision/Pose2dCamera", cameraPose);
    Logger.recordOutput("Vision/Pose2d", pose);

    return pose;
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
