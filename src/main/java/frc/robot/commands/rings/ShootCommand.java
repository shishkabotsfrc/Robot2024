package frc.robot.commands.rings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ForceEject;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends SequentialCommandGroup {
  public ShootCommand(Shooter m_shooter, Intake intake) {
    addCommands(new PrimeShooter(m_shooter), new ForceEject(intake));
  }
}
