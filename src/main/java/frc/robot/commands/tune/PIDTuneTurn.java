package frc.robot.commands.tune;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleId;
import frc.utils.CurrentTime;
import frc.utils.StepResponseAnalyzer;
import org.littletonrobotics.junction.Logger;

public class PIDTuneTurn extends Command {

  private static final double DELTA = 0.1;
  private static final double MAX_TIME_MS = 5000.;
  private static final double MIN_INIT_TIME_MS = 1000.;
  private static final double START_ANGLE = Math.PI / 4;
  private static final double TARGET_ANGLE = Math.PI / 2. + Math.PI / 4;

  private DriveSubsystem m_drive;
  private double m_startAngle;
  private double m_finalAngle;
  private double m_startTime = 0.;
  private double m_startStepTime = 0.;

  private Phase m_state;

  private AngleDriver m_angleDriver;
  StepResponseAnalyzer[] m_analyzers = new StepResponseAnalyzer[4];

  private enum PhaseStatus {
    WORKING,
    SUCCESS,
    FAILED
  };

  private enum Phase {
    INITIALIZE,
    STEP,
    DONE,
    FAILED
  };

  // Step from 45 degrees to 135 to avoid any angle wra around region
  public PIDTuneTurn(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_startAngle = START_ANGLE;
    m_finalAngle = TARGET_ANGLE;
  }

  public void initialize() {
    m_state = Phase.INITIALIZE;
    m_startTime = CurrentTime.millis();
    m_angleDriver = new AngleDriver(m_drive, DELTA);
    m_angleDriver.setTargetAngle(m_startAngle);
    for (int i = 0; i < 4; i++) {
      m_analyzers[i] = new StepResponseAnalyzer(m_startAngle, m_finalAngle, DELTA);
    }
  }

  private double elapsedTime() {
    return CurrentTime.millis() - m_startTime;
  }

  private double stepTime() {
    return CurrentTime.millis() - m_startStepTime;
  }

  private PhaseStatus initPhaseStatus() {
    if (elapsedTime() > MAX_TIME_MS) {
      return PhaseStatus.FAILED;
    } else if (m_angleDriver.isAllAtTarget() && elapsedTime() >= MIN_INIT_TIME_MS) {
      return PhaseStatus.SUCCESS;
    } else {
      return PhaseStatus.WORKING;
    }
  }

  private PhaseStatus stepPhaseStatus() {
    if (elapsedTime() > MAX_TIME_MS) {
      return PhaseStatus.FAILED;
    }

    if (!m_angleDriver.isAllAtTarget()) {
      return PhaseStatus.WORKING;
    }
    for (int i = 0; i < 4; i++) {
      if (!m_analyzers[i].isSettled()) {
        return PhaseStatus.WORKING;
      }
    }
    return PhaseStatus.SUCCESS;
  }

  public void execute() {

    if (isFinished()) {
      return;
    }

    m_angleDriver.drive();
    // Log the values
    Logger.recordOutput("DebugPidTurn/Values/target", m_angleDriver.getTargetAngle());
    for (int i = 0; i < 4; i++) {
      String module = ModuleId.fromInt(i).toString();
      Logger.recordOutput("DebugPidTurn/Values/" + module, m_angleDriver.getAngle(i));
    }

    double time = stepTime();
    if (m_state == Phase.STEP) {
      for (int i = 0; i < 4; i++) {
        m_analyzers[i].update(time, m_angleDriver.getAngle(i));
      }
    }

    // Main logic
    switch (m_state) {
      case INITIALIZE:
        PhaseStatus initStatus = initPhaseStatus();
        if (initStatus == PhaseStatus.FAILED) {
          m_state = Phase.FAILED;
        } else if (initStatus == PhaseStatus.SUCCESS) {
          m_state = Phase.STEP;
          m_startStepTime = CurrentTime.millis();
          m_angleDriver.setTargetAngle(m_finalAngle);
          for (int i = 0; i < 4; i++) {
            m_analyzers[i].update(0., m_angleDriver.getAngle(i));
          }
        }
        break;
      case STEP:
        PhaseStatus stepStatus = stepPhaseStatus();
        if (stepStatus == PhaseStatus.FAILED) {
          m_state = Phase.FAILED;
        } else if (stepStatus == PhaseStatus.SUCCESS) {
          reportResults();
          m_state = Phase.DONE;
        }
        break;
      default:
        return;
    }

    Logger.recordOutput("DebugPidTurn/State", m_state.toString());
  }

  private void reportResults() {
    for (int i = 0; i < 4; i++) {
      String module = ModuleId.fromInt(i).toString();
      Logger.recordOutput("DebugPidTurn/FinalValue/" + module, m_analyzers[i].finalValue());
      Logger.recordOutput(
          "DebugPidTurn/SSError/" + module, m_analyzers[i].finalValue() - m_finalAngle);
      Logger.recordOutput("DebugPidTurn/Overshoot/" + module, m_analyzers[i].overshoot());
      Logger.recordOutput("DebugPidTurn/SettlingTime/" + module, m_analyzers[i].settlingTime());
      // Logger.recordOutput("DebugPidTurn/RiseTime/" + module, "TODO");
    }
  }

  @Override
  public boolean isFinished() {
    return m_state.equals(Phase.DONE) || m_state.equals(Phase.FAILED);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_state = Phase.FAILED;
    }
  }
}
