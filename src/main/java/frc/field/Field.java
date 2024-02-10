package frc.field;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.HashMap;

public class Field {
  public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();

  static {
    kAprilTagMap.put(586, new AprilTag(586, new Pose3d(2.0, 3, 0.9, new Rotation3d())));
  }
}
