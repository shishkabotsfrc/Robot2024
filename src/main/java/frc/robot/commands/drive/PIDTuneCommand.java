package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.CurrentTime;
import frc.utils.SwerveUtils;

public class PIDTuneCommand extends Command {
  private static final double NOT_SET_TIME = 0.0;
  private static final double SETTLE_TIME = 1.0;

  private DriveSubsystem drive;
  private double targetAngle;
  private double secondsSinceTargetSet;
  private double timeWhenTargetReached;
  private State state;

  private enum State {
    INITIALIZE,
    STEP,
    DONE,
  };

  public PIDTuneCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  public void initialize() {
    state = State.INITIALIZE;
    setTargetRotation(Math.PI);
  }

  private void setTargetRotation(double radians) {
    secondsSinceTargetSet = CurrentTime.seconds();
    timeWhenTargetReached = NOT_SET_TIME;
    targetAngle = radians;
  }

  private boolean isSettled() {
    if (timeWhenTargetReached == NOT_SET_TIME) return false;
    return (CurrentTime.seconds() - timeWhenTargetReached) > SETTLE_TIME;
  }

  public void execute() {
    SwerveModuleState[] targetSwerveStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      targetSwerveStates[i] = new SwerveModuleState(0.0, new Rotation2d(targetAngle));
    }
    drive.setModuleStates(targetSwerveStates);

    // Motors may never reach desired state so abort after 3s
    if (CurrentTime.seconds() - secondsSinceTargetSet > 3.0) {
      System.out.println("[PID Tune] Aborting command due to timeout");
      state = State.DONE;
    }
    // How many of the modules are within the threshold
    int reachedModules = 0;
    for (int i = 0; i < 4; i++) {
      double moduleAngle = SwerveUtils.WrapAngle(drive.modules[i].getState().angle.getRadians());
      double angleDifference = SwerveUtils.AngleDifference(moduleAngle, targetAngle);
      drive.modules[i].setDesiredState(new SwerveModuleState(0.0, new Rotation2d(targetAngle)));
      if (Math.abs(angleDifference) < 0.1) {
        reachedModules += 1;
      }
    }
    // If all modules reached, then update `timeWhenTargetReached`
    // TODO: some will reverse, so angle will be off by 180 degrees
    if (reachedModules > 0) {
      if (timeWhenTargetReached == NOT_SET_TIME) {
        timeWhenTargetReached = CurrentTime.seconds();
      }
    } else {
      timeWhenTargetReached = NOT_SET_TIME;
    }
    // Zeros the angle then times
    switch (state) {
      case INITIALIZE:
        if (isSettled()) {
          setTargetRotation(Math.PI / 2);
          state = State.STEP;
        }
      case STEP:
        if (isSettled()) {
          state = State.DONE;

          System.out.println(
              "[PID Tune] seconds to settle: "
                  + (CurrentTime.seconds() - secondsSinceTargetSet - SETTLE_TIME));
        }
      default:
        break;
    }
  }

  public boolean isFinished() {
    return state.equals(State.DONE);
  }
}
