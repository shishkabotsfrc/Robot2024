package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotTarget;

import frc.robot.subsystems.Intake.IntakeState;

public class FeedIntake extends Command {
  private Intake m_intake;
  private int counter = 0;

  public FeedIntake(Intake intake) {
    // TODO:choose one later after testing
    // m_intake = Intake.getInstance();
    m_intake = intake;
  }

  @Override
  public void initialize() {
    m_intake.setState(IntakeState.INTAKE);
    //m_intake.setPivotTarget(PivotTarget.GROUND);
    //m_intake.goToGround();
  }

  public void execute() {
    m_intake.periodic();
    // counter++;
  }

  @Override
  public boolean isFinished() {
    /*if (counter > 200
        && (m_intake.getIntakeState() == IntakeState.INTAKE
            || m_intake.getPivotTarget() == PivotTarget.GROUND)) {
      return true;
    } else {
      return false;
    }*/
    return false;
  }
}
