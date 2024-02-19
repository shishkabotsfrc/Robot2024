package frc.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagInfo {
  public int id;
  public Translation3d pose;
  public Rotation3d rotation;
  public MarkerType type;
  public Alliance alliance;

  public enum MarkerType {
    Chain,
    SpeakerCenter,
    SpeakerOffCenter,
    RingSourceLeft,
    RingSourceRight,
    Amplifier
  }

  public AprilTagInfo(
      int id, Alliance alliance, MarkerType type, Translation3d pose, double rotation) {
    this.id = id;
    this.alliance = alliance;
    this.type = type;
    this.pose = pose;
    this.rotation = new Rotation3d(0, 0, rotation);
  }

  // TODO: double check that this does not allow mutation
  public Pose3d pose() {
    return new Pose3d(pose, rotation);
  }
}
