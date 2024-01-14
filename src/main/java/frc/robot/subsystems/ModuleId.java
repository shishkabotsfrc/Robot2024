package frc.robot.subsystems;

public enum ModuleId {
  FL(0),
  FR(1),
  BR(2),
  BL(3);
  private int m_index;

  ModuleId(int index) {
    this.m_index = index;
  }

  int index() {
    return this.m_index;
  }
}
