package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotTarget;
import frc.robot.subsystems.Shooter;

public class FeedIntake extends Command {
  private Intake m_intake;
  private Shooter m_shooter;
  private boolean finish = false;
  private int gotNote = 0;

  public FeedIntake(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  public FeedIntake(Intake intake, Shooter shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(m_intake, m_shooter);
  }

  @Override
  public void initialize() {
    finish = false;
    System.out.println("[feedintake] init");
    gotNote = 0;
    m_intake.goToGround();
  }

  public void execute() {

    if (m_intake.getIntakeHasNote()) {
      System.out.println("hello");
      gotNote++;
      if (gotNote >= 10) {
        m_intake.goToStow();
      }
    }
    if (m_intake.isPivotAtTarget(PivotTarget.STOW) && gotNote >= 3) {
      // m_intake.setState(IntakeState.PULSE);
      finish = true;
    }
  }

  @Override
  public boolean isFinished() {
    return finish;
  }
}

// stop intake if shooter not ready
