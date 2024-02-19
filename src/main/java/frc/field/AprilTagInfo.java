package frc.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagInfo {
  private int m_id;
  private Pose3d m_pose;
  private MarkerType m_type;
  private Alliance m_alliance;

  public enum MarkerType {
    Chain,
    SpeakerCenter,
    SpeakerOffCenter,
    RingSourceLeft,
    RingSourceRight,
    Amplifier
  }

  /**
   * Creates a new Apriltag object
   *
   * @param id the number associated with the tag
   * @param alliance team color of marked object
   * @param type type of game object is being marked
   * @param translation translation (meters)
   * @param rotation rotation (radians)
   */
  public AprilTagInfo(
      int id, Alliance alliance, MarkerType type, Translation3d translation, double rotation) {
    m_id = id;
    m_alliance = alliance;
    m_type = type;
    m_pose = new Pose3d(translation, new Rotation3d(0, 0, rotation));
  }

  public int id() {
    return m_id;
  }

  /** Returns a copy of the Apriltag pose */
  public Pose3d pose() {
    // TODO: Is there a one-liner to make a copy?
    Rotation3d rotation = m_pose.getRotation();
    Rotation3d rotCopy = new Rotation3d(rotation.getX(), rotation.getY(), rotation.getZ());
    return new Pose3d(m_pose.getX(), m_pose.getY(), m_pose.getZ(), rotCopy);
  }

  public MarkerType type() {
    return m_type;
  }

  public Alliance alliance() {
    return m_alliance;
  }
}
