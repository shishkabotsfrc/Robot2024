package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.detectColor;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double k_pivotMotorP = 0.12;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.001;

  private final PIDController m_pivotPID =
      new PIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD);

  private final DutyCycleEncoder m_pivotEncoder =
      new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);
  // private final DigitalInput m_IntakeLimitSwitch =
  //  new DigitalInput(Constants.Intake.k_intakeLimitSwitchId);

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;

  public Intake() {
    // super("Intake");

    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(10);

    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  // @Override
  public void periodic() {
    checkAutoTasks();

    // Pivot control
    double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);

    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    if (m_pivotEncoder.get() == 0.0) {
      m_periodicIO.intake_pivot_voltage = 0.0;
    }

    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
    Logger.recordOutput("State", m_periodicIO.intake_state.toString());
    writePeriodicOutputs();
    outputTelemetry();
  }

  // @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  // @Override
  public void stop() {
    m_periodicIO.intake_pivot_voltage = 0.0;
    m_periodicIO.intake_speed = 0.0;
  }

  // @Override
  public void outputTelemetry() {
    Logger.recordOutput("Pivot/Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    Logger.recordOutput("Pivot/Abs Enc (get)", m_pivotEncoder.get());
    Logger.recordOutput(
        "Pivot/Abs Enc (getAbsolutePosition)", m_pivotEncoder.getAbsolutePosition());
    Logger.recordOutput("Pivot/Abs Enc (getPivotAngleDegrees)", getPivotAngleDegrees());
    Logger.recordOutput("Pivot/Setpoint", pivotTargetToAngle(m_periodicIO.pivot_target));

    Logger.recordOutput("Pivot/Power", m_periodicIO.intake_pivot_voltage);
    Logger.recordOutput("Pivot/Current", mPivotMotor.getOutputCurrent());

    Logger.recordOutput("Limit Switch", getIntakeHasNote());
  }

  // @Override
  public void reset() {}

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case SOURCE:
        return Constants.Intake.k_pivotAngleSource;
      case AMP:
        return Constants.Intake.k_pivotAngleAmp;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
      default:
        // "Safe" default
        return 180;
    }
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return Constants.Intake.k_intakeSpeed;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

  public double getPivotAngleDegrees() {
    double value =
        m_pivotEncoder.getAbsolutePosition() - Constants.Intake.k_pivotEncoderOffset + 0.5;

    return Units.rotationsToDegrees(modRotations(value));
  }

  public static double modRotations(double input) {
    input %= 1.0;
    if (input < 0.0) {
      input += 1.0;
    }
    return input;
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    detectColor detColor = new detectColor();
    return detColor.gotNote();
  }

  // Pivot helper functions
  public void goToGround() {
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void goToSource() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToAmp() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToStow() {
    m_periodicIO.pivot_target = PivotTarget.STOW;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND
        && getIntakeHasNote()
        && isPivotAtTarget()) {
      m_periodicIO.pivot_target = PivotTarget.STOW;
      m_periodicIO.intake_state = IntakeState.NONE;
    }
  }

  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }

  public PivotTarget getPivotTarget() {
    return m_periodicIO.pivot_target;
  }
}
