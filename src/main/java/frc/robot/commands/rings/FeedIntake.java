package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class FeedIntake extends Command {
  private Intake m_intake;
  private IntakeState m_intakeState;
  private PivotTarget m_pivotTarget;

  // IntakeCommand(Intake, ground)
  // (feedshooter, none)
  //
  public FeedIntake(IntakeState intakeState, PivotTarget pivotTarget) {
    m_intake = Intake.getInstance();
    m_intakeState = intakeState;
    m_pivotTarget = pivotTarget;
  }

  @Override
  public void initialize() {
    m_intake.setState(m_intakeState);
    m_intake.setPivotTarget(m_pivotTarget);
    m_intake.goToGround();
  }

  @Override
  public void execute() {
    m_intake.periodic();
  }
}
