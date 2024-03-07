package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
  private final Shooter m_shooter;
  private Intake m_intake;
  private int counter = 0;

  public ShootCommand(Shooter shooter, Intake intake) {
    this.m_shooter = shooter;
    this.m_intake = intake;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // m_intake.feedShooter();
  }

  @Override
  public void execute() {
    // Only press if fully loaded?
    // if (m_intake.getIntakeState().equals(IntakeState.FEED_SHOOTER)) {
    m_shooter.setSpeed(5700 / 3);
    counter++;
    // }
  }

  @Override
  public boolean isFinished() {
    if (counter > 300) {
      m_shooter.stop();
      m_intake.setState(IntakeState.NONE);
      m_intake.setPivotTarget(PivotTarget.STOW);
      return true;
    } else {
      return false;
    }
  }
}
