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
  private static final double THRESHOLD = 0.1;
  private static double TARGET_ROTATION = Math.PI / 2;
  private static int ITERATIONS = 4;

  private DriveSubsystem drive;
  private double targetAngle;
  private double secondsSinceTargetSet;
  private double timeWhenTargetReached;
  private double totalTimeToReach;
  private int state;

  public PIDTuneCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  public void initialize() {
    state = 0;
    totalTimeToReach = 0.0;
    setTargetRotation(0.0);
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
    // Motors may never reach desired state so abort after 3s
    if (CurrentTime.seconds() - secondsSinceTargetSet > 3.0) {
      System.err.println("[PID Tune] Aborting command due to 3s timeout");
      state = ITERATIONS;
      return;
    }
    // How many of the modules are within the threshold
    int reachedModules = 0;
    for (int i = 0; i < 4; i++) {
      drive.modules[i].setDesiredState(new SwerveModuleState(0.0, new Rotation2d(targetAngle)));
      double moduleAngle = SwerveUtils.WrapAngle(drive.modules[i].getAngle());
      double angleDifference = SwerveUtils.AngleDifference(moduleAngle, targetAngle);
      if (Math.abs(angleDifference) < THRESHOLD) {
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
    if (isSettled()) {
      if (state != 0) {
        totalTimeToReach += CurrentTime.seconds() - secondsSinceTargetSet - SETTLE_TIME;
      }
      state += 1;
      if (isFinished()) {
        System.out.println("[PID Tune] Average time: " + totalTimeToReach / ITERATIONS);
      } else {
        setTargetRotation(TARGET_ROTATION * state);
      }
    }
  }

  public boolean isFinished() {
    return (state == ITERATIONS);
  }
}
