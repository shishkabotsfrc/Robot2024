package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  private Climber climber;

  public Climb(Climber climber) {
    this.climber = climber;
  }

  public void execute() {
    climber.climb(600);
  }
}
