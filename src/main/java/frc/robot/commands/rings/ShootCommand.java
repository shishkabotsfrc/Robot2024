package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
  private final Shooter m_shooter;

  public ShootCommand(Shooter shooter) {
    this.m_shooter = shooter;
    addRequirements(shooter);
  }

  public void execute() {
    // TODO
  }
}
