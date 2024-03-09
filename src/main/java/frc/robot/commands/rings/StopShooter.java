package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
  private Shooter m_shooter;

  public StopShooter(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setSpeed(0);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
