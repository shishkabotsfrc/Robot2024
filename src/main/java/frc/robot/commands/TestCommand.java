package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Sample command to illustrate the sample use. */
public class TestCommand extends Command {

  int m_counter;
  boolean m_finishes;

  public TestCommand(Subsystem subsystem, boolean finishes) {
    System.out.println("TEST_COMMAND: constructor");
    addRequirements(subsystem);
    m_counter = 0;
    m_finishes = finishes;
  }

  @Override
  public void initialize() {
    System.out.println("TEST_COMMAND: initialize");
    m_counter = 0;
  }

  @Override
  public void execute() {
    m_counter++;
    // Log every 50 interations (1 second).
    if (m_counter % 50 == 0) {
      System.out.println("TEST_COMMAND: execute: " + Integer.toString(m_counter));
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TEST_COMMAND: end: interrupted: " + Boolean.toString(interrupted));
  }

  @Override
  public boolean isFinished() {
    // End the command after 250 iterations (5 seconds)
    if (m_counter >= 250 && m_finishes) {
      System.out.println("TEST_COMMAND: isFinished num iterations: " + Integer.toString(m_counter));
      return true;
    } else {
      return false;
    }
  }
}
