package frc.robot.subsystems.drive;

/** Enum that maps the index in the swerve module array to a chassis corner. */
public enum ModuleId {
  FL(0),
  FR(1),
  RL(2),
  RR(3);
  private int m_index;

  ModuleId(int index) {
    this.m_index = index;
  }

  /**
   * Returns the corresponding index of the enum.
   *
   * @return index of the enum.
   */
  public int index() {
    return this.m_index;
  }

  /**
   * Returns the enum that corresponds to the index
   *
   * @param index The index
   * @return The corresponding enum
   * @throws
   */
  public static ModuleId fromInt(int index) {

    switch (index) {
      case (0):
        return FL;
      case (1):
        return FR;
      case (2):
        return RL;
      case (3):
        return RR;
      default:
        throw new IllegalArgumentException("Index is out of range.");
    }
  }
}
