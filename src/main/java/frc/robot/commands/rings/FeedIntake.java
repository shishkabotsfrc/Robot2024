package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotTarget;
import frc.robot.subsystems.Shooter;

public class FeedIntake extends Command {
  private Intake m_intake;

  private Shooter m_shooter;
  private int counter = 0;

  public FeedIntake(Intake intake, Shooter shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(m_intake, m_shooter);
  }

  @Override
  public void initialize() {
    m_intake.setPivotTarget(PivotTarget.GROUND);
  }

  public void execute() {
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
    // if (counter > 100) {
    //   m_intake.setState(IntakeState.PULSE);
    // }
    return false;
  }
}

// stop intake if shooter not ready
