package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class ShootAmp extends Command {

  private Intake m_intake;
  private int counter = 0;

  public ShootAmp(Intake intake) {
    // TODO:choose one later after testing
    // m_intake = Intake.getInstance();
    m_intake = intake;
  }

  @Override
  public void initialize() {
    m_intake.setState(IntakeState.EJECT);
    m_intake.setPivotTarget(PivotTarget.AMP);
    m_intake.goToAmp();
  }

  @Override
  public void execute() {
    if (m_intake.getIntakeState() == IntakeState.EJECT) {
      m_intake.periodic();
      counter++;
    }
  }

  @Override
  public boolean isFinished() {
    if (counter > 200) {
      m_intake.setState(IntakeState.NONE);
      m_intake.setPivotTarget(PivotTarget.STOW);
      return true;
    } else {
      return false;
    }
  }
}
