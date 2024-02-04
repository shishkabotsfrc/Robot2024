package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.field.Field;

public class DetectedTarget {
  private Pose3d tagPoseRelative = new Pose3d();
  private Limelight.PeriodicIO periodicIO;
  private int i = 0;
  public void update(Limelight.PeriodicIO inputs) {
    periodicIO = inputs;
  }

  public Pose3d getPoseWorld(Pose3d robot) {
    // AprilTag tag = Field.kAprilTagMap.get((int) periodicIO.targetID);
    // if (tag == null) {
    //   System.err.println("Uknown tag: " + periodicIO.targetID);
    //   return null;
    // }
    // TODO: register all tag position
    double heightDiff = (5);
    double distance_2d = heightDiff / Math.tan(periodicIO.ty);
    double x = Math.cos(periodicIO.tx) * distance_2d;
    double y = Math.sin(periodicIO.ty) * distance_2d;
      if (i % 5 == 0) System.out.println(x + "  " + y);
      i+=1;

    return tagPoseRelative;
  }
}
