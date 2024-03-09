package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
  private Shooter m_shooter;
  private int counter = 0;

  public StopShooter(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    counter=0;
    m_shooter.setSpeed(0);
  }

  @Override
  public void execute() {
    counter++;
  }

  @Override
  public boolean isFinished() {
   
    if(counter>25) {
      return true;
    }
    return false;
  }
}
