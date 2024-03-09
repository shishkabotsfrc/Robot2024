package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.PivotTarget;

public class StowIntake extends Command {
  private Intake m_intake;
  private int counter;

  public StowIntake(Intake intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    counter=0;
    m_intake.setPivotTarget(PivotTarget.STOW);
  }

  @Override
  public void execute() {
counter++;
  }

  @Override
  public boolean isFinished() {
    if(counter>50) {
    return true;
    }
    return false;
  }
}
