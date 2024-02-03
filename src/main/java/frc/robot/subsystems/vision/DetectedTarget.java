package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.field.Field;

public class DetectedTarget {
  private Pose3d tagPoseRelative;
  private Limelight.PeriodicIO periodicIO;

  public void update(Limelight.PeriodicIO inputs) {
    periodicIO = inputs;
  }

  public Pose3d getPoseWorld(Pose3d robot) {
    AprilTag tag = Field.kAprilTagMap.get((int) periodicIO.targetID);
    if (tag == null) {
      System.err.println("Uknown tag: " + periodicIO.targetID);
      return null;
    }
    double distance_2d = (tag.pose.getY() - 10) / Math.tan(periodicIO.ty);
    // TODO: math
    return tagPoseRelative;
  }
}
