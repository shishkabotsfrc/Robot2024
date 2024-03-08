package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;
import frc.robot.subsystems.Shooter;

public class FeedIntake extends Command {
  private Intake m_intake;
  private Shooter m_shooter;

  private int gotNote = 0;

  public FeedIntake(Intake intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  public FeedIntake(Intake intake, Shooter shooter) {
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(m_intake, m_shooter);
  }

  @Override
  public void initialize() {
    System.out.println("[feedintake] init");
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
    if (m_intake.isPivotAtTarget(PivotTarget.STOW) && gotNote >= 10) {
      m_intake.setState(IntakeState.PULSE);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

// stop intake if shooter not ready
