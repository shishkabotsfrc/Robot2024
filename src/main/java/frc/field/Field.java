package frc.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.field.AprilTagInfo.MarkerType;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** Represents the playing field, everything notable is marked by apriltags */
public class Field {
  public static final HashMap<Integer, AprilTagInfo> kAprilTagMap = new HashMap<>();
  private static final HashMap<MarkerType, ArrayList<Transform2d>> kObjectives = new HashMap<>();

  static {
    // Location of all apriltags and thus objectives
    // See page 4 for additional documentation:
    // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    addTag(1, Alliance.Blue, MarkerType.RingSourceLeft, 593.68, 9.68, 53.38, 120);
    addTag(2, Alliance.Blue, MarkerType.RingSourceRight, 637.21, 34.79, 53.38, 120);
    addTag(3, Alliance.Red, MarkerType.SpeakerOffCenter, 652.73, 196.17, 57.13, 180);
    addTag(4, Alliance.Red, MarkerType.SpeakerCenter, 652.73, 218.42, 57.13, 180);
    addTag(5, Alliance.Red, MarkerType.Amplifier, 578.77, 323.00, 53.38, 270);
    addTag(6, Alliance.Blue, MarkerType.Amplifier, 72.5, 323.00, 53.38, 270);
    addTag(7, Alliance.Blue, MarkerType.SpeakerCenter, -1.50, 218.42, 57.13, 0);
    addTag(8, Alliance.Blue, MarkerType.SpeakerOffCenter, -1.50, 196.17, 57.13, 0);
    addTag(9, Alliance.Blue, MarkerType.RingSourceLeft, 14.02, 34.79, 53.38, 60);
    addTag(10, Alliance.Blue, MarkerType.RingSourceRight, 57.54, 9.68, 53.38, 60);
    addTag(11, Alliance.Red, MarkerType.Chain, 468.69, 146.19, 52.00, 300);
    addTag(12, Alliance.Red, MarkerType.Chain, 468.69, 177.10, 52.00, 60);
    addTag(13, Alliance.Red, MarkerType.Chain, 441.74, 161.62, 52.00, 180);
    addTag(14, Alliance.Blue, MarkerType.Chain, 209.48, 161.62, 52.00, 0);
    addTag(15, Alliance.Blue, MarkerType.Chain, 182.73, 177.10, 52.00, 120);
    addTag(16, Alliance.Blue, MarkerType.Chain, 182.73, 146.19, 52.00, 240);
  }

  static {
    // Location of all scoring locations
    addTarget(MarkerType.Amplifier, 0.0, 0.0, 0.0);

    // for (MarkerType marker : MarkerType.values()) {
    //   if (!kObjectives.containsKey(marker)) {
    //        TODO: Silenced for now
    //        System.err.println(marker + " has not yet been registered");
    //   }
    // }
  }

  /** Returns an AprilTag position based on the given id, will report errors if tag is invalid */
  public static AprilTagInfo getTag(int id) {
    AprilTagInfo tag = kAprilTagMap.get(id);
    if (tag == null) {
      System.err.println(id + " is not found as an apriltag");
      return null;
    }
    if (tag.id() != id) {
      System.err.println(id + " does not match april tag id: " + tag.id());
      return null;
    }
    return tag;
  }

  public static ArrayList<Transform2d> getTargetTranslations(MarkerType marker) {
    ArrayList<Transform2d> targets = kObjectives.get(marker);
    if (targets == null) {
      System.err.println(marker + " has no associated translations with it");
      var ret = new ArrayList<Transform2d>();
      ret.add(new Transform2d());
      return ret;
    }
    return targets;
  }

  /** Filters all Apriltags which match given criteria */
  public static ArrayList<AprilTagInfo> getAllTagsByType(Alliance alliance, MarkerType type) {
    ArrayList<AprilTagInfo> tagList = new ArrayList<>();
    for (AprilTagInfo tag : kAprilTagMap.values()) {
      if (DriverStation.getAlliance().get() == alliance && type.equals(tag.type())) {
        tagList.add(tag);
      }
    }
    if (tagList.size() == 0) {
      System.err.println("There were no tags detected that are " + alliance + type);
    }
    return tagList;
  }

  /** Returns the best matching objective by the filters. Place to drive to do task */
  public static Pose2d getClosestObjectivePoseByType(
      Pose2d robot, Alliance alliance, List<MarkerType> markerTypes) {
    double bestDistance = 1e7;
    Pose2d bestPose = null;
    for (MarkerType marker : markerTypes) {
      for (AprilTagInfo tag : getAllTagsByType(alliance, marker)) {
        for (Transform2d translate : getTargetTranslations(marker)) {
          // TODO: Does this move offset forward from perspective of game piece?
          Pose2d pose = tag.pose().toPose2d().plus(translate);
          double distance = pose.getTranslation().getDistance(robot.getTranslation());
          if (distance < bestDistance) {
            bestDistance = distance;
            bestPose = pose;
          }
        }
      }
    }
    if (bestPose == null) {
      System.err.println(markerTypes + " " + alliance + " returned null pose");
    }
    return bestPose;
  }

  /** Adds a position for the robot to travel to before completing an objective */
  private static void addTarget(MarkerType type, double x, double y, double rot) {
    // Init the arraylist so doesnt write to null
    if (!kObjectives.containsKey(type)) {
      kObjectives.put(type, new ArrayList<>());
    }
    Transform2d transform = new Transform2d(x, y, new Rotation2d(rot));
    kObjectives.get(type).add(transform);
  }

  /** Adds a tag to the map of all known tags */
  private static void addTag(
      int id, Alliance alliance, MarkerType type, double x, double y, double z, double rot) {
    var pose = new Translation3d(inchToMeter(x), inchToMeter(y), inchToMeter(z));
    double rotation = Units.degreesToRadians(rot);
    kAprilTagMap.put(id, new AprilTagInfo(id, alliance, type, pose, rotation));
  }

  private static double inchToMeter(double inch) {
    return inch * 0.0254;
  }
}
