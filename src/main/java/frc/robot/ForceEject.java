package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class ForceEject extends Command {
  Intake m_intake;

  public ForceEject(Intake intake) {
    m_intake = intake;
  }

  public void initialize() {
    m_intake.setState(IntakeState.EJECT);
  }
}
