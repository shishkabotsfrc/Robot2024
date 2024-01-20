package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class PIDTuneCommand extends Command {
  private DriveSubsystem drive;
  private int counter = 0;
  private int counter2 = 0;
  private int check = 0;
  double startTime = WPIUtilJNI.now() * 1e-6;
  private SwerveModuleState[] swerveState;
  private State state;

  private enum State {
    INITIALIZE,
    STEP,
    DONE,
    SETTLED;
  };

  public PIDTuneCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    swerveState = new SwerveModuleState[4];
    System.out.println("PID: Constructor");
  }

  public void initialize() {
    System.out.println("PID: Initialize");

    counter = 0;
    state = State.INITIALIZE;
    for (int i = 0; i < 4; i++) {
      swerveState[i] = new SwerveModuleState(0.0, new Rotation2d(0.));
    }
  }

  public void execute() {

    counter++;

    switch (state) {
      case INITIALIZE:
        drive.setModuleStates(swerveState);
        if (counter >= 50) {
          for (int i = 0; i < 4; i++) {
            swerveState[i] = new SwerveModuleState(0.0, new Rotation2d(Math.PI / 2));
          }
          System.out.println("PID: Step");
          state = State.STEP;
        }
        break;

      case STEP:
        drive.setModuleStates(swerveState);

        check = 0;
        for (int i = 0; i < swerveState.length; i++) {
          if ((Math.abs(drive.modules[i].getState().angle.getRadians() - Math.PI / 2))
              < (0.05 * Math.PI / 2)) {
            check++;
          }
        }
        if (check == 4) {
          counter2++;
        } else {
          counter2 = 0;
        }
        if (counter2 >= 50) {
          System.out.println("PID: Done");
          state = State.DONE;
        }

        break;

      case DONE:
        break;

      case SETTLED:
        break;
    }

    // drive.setModuleStates(swerveState);

    // if (((Math.abs(drive.m_frontLeft.getState().angle.getRadians() - Math.PI / 2))) < 0.1) {
    //   counter++;
    // } else {
    //   counter = 0;
    // }

    // counter ++;
    // if (counter > 20) {
    //   System.out.println((WPIUtilJNI.now() * 1e-6) - startTime);
    // }
  }

  public boolean isFinished() {
    if (state.equals(State.DONE)) {
      return true;
    } else {
      return false;
    }
  }
}
