package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
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

  public void initialize() {
    m_intake.feedShooter();
  }

  @Override
  public void execute() {
    // Only press if fully loaded?
    if (m_intake.getIntakeState().equals(IntakeState.FEED_SHOOTER)) {
      m_shooter.setSpeed(10);
      counter++;
    }
  }

  @Override
  public boolean isFinished() {
    if (counter > 100) {
      m_shooter.stop();
      m_intake.setState(IntakeState.NONE);
      return true;
    } else {
      return false;
    }
  }
}