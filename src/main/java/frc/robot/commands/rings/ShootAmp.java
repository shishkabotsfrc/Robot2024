package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotTarget;

public class ShootAmp extends Command {

  private Intake m_intake;
  private int counter = 0;

  public ShootAmp(Intake intake) {
    // TODO:choose one later after testing
    // m_intake = Intake.getInstance();
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.setPivotTarget(PivotTarget.AMP);
  }

  @Override
  public void execute() {
    counter++;
  }

  @Override
  public boolean isFinished() {
    if (counter > 50) {
      return true;
    }
    return false;
  }
}
