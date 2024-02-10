package frc.field;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

public class Field {
  public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();

  static {
    // See page 4 for additional documentation:
    // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    addTag(1, 593.68, 9.68, 53.38, 120);
    addTag(2, 637.21, 34.79, 53.38, 120);
    addTag(3, 652.73, 196.17, 57.13, 180);
    addTag(4, 652.73, 218.42, 57.13, 180);
    addTag(5, 578.77, 323.00, 53.38, 270);
    addTag(6, 72.5, 323.00, 53.38, 270);
    addTag(7, -1.50, 218.42, 57.13, 0);
    addTag(8, -1.50, 196.17, 57.13, 0);
    addTag(9, 14.02, 34.79, 53.38, 60);
    addTag(10, 57.54, 9.68, 53.38, 60);
    addTag(11, 468.69, 146.19, 52.00, 300);
    addTag(12, 468.69, 177.10, 52.00, 60);
    addTag(13, 441.74, 161.62, 52.00, 180);
    addTag(14, 209.48, 161.62, 52.00, 0);
    addTag(15, 182.73, 177.10, 52.00, 120);
    addTag(16, 182.73, 146.19, 52.00, 240);
  }

  private static void addTag(int id, double x, double y, double z, double rot) {
    var pose =
        new Pose3d(
            inchToMeter(x),
            inchToMeter(y),
            inchToMeter(z),
            new Rotation3d(0, 0, Units.degreesToRadians(rot)));
    kAprilTagMap.put(id, new AprilTag(id, pose));
  }

  private static double inchToMeter(double inch) {
    return inch * 0.0254;
  }
}
