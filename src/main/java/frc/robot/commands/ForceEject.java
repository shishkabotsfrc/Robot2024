package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class ForceEject extends Command {
  private Intake m_intake;
  private int counter;

  public ForceEject(Intake intake) {
    m_intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    counter = 0;
    m_intake.setState(IntakeState.EJECT2);
  }

  @Override
  public void execute() {
    // if (counter >= 10) {
    //   m_intake.setState(IntakeState.PULSE);
    // }
    counter++;
  }

  @Override
  public boolean isFinished() {
    // if (!(m_intake.getIntakeHasNote()) && counter > 60) {
    //   return true;
    // }
    if (counter > 70) {
      return true;
    }
    return false;
  }
}
