package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

public class PrimeShooter extends Command {
  private Shooter m_Shooter;

  public PrimeShooter(Shooter shooter) {
    m_Shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_Shooter.setSpeed(ShooterConstants.kTargetRPM);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
