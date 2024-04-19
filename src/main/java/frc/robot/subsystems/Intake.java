package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.ColorSensor;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double kPivotEncoderPositionFactor =
      360.0; // degrees, Should this be radians?
  private static final double kPivotEncoderVelocityFactor = 360. / 60.0; // degrees per second
  private static final ColorSensor mcolorDetector = new ColorSensor();
  // // private static Intake mInstance;
  private PeriodicIO m_periodicIO;
  // // public static Intake getInstance() {
  // //   if (mInstance == null) {
  // //     mInstance = new Intake();
  // //   }
  // //   return mInstance;
  // // }

  private CANSparkMax mIntakeMotor;
  private RelativeEncoder mIntakeEncoder;
  private SparkMaxPIDController mIntakePIDController;

  private CANSparkMax mPivotMotor;
  private AbsoluteEncoder mPivotEncoder;
  private SparkMaxPIDController mPivotPIDController;

  public Intake() {

    //   // Intake motor (same as the driving motor from max swerve)
    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mIntakeMotor.setSmartCurrentLimit(15);

    mIntakeEncoder = mIntakeMotor.getEncoder();

    //   // TODO: Figure out the constants
    // mIntakeEncoder.setPositionConversionFactor(1.);
    // mIntakeEncoder.setVelocityConversionFactor(1.);

    // mIntakeMotor.burnFlash();

    // Intake motor (same as the turning motor from max swerve)
    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(25);

    mPivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    mPivotPIDController = mPivotMotor.getPIDController();
    mPivotPIDController.setFeedbackDevice(mPivotEncoder);

    mPivotPIDController.setP(1.0);
    mPivotPIDController.setI(0.00020);
    mPivotPIDController.setD(0);
    // mPivotPIDController.setFF(0);
    // mPivotPIDController.setOutputRange(-1., 1.);

    // // Do we need this
    // mPivotPIDController.setPositionPIDWrappingEnabled(true); // This may not be needed
    // mPivotPIDController.setPositionPIDWrappingMinInput(0.);
    // mPivotPIDController.setPositionPIDWrappingMaxInput(kPivotEncoderPositionFactor);

    // mPivotEncoder.setPositionConversionFactor(kPivotEncoderPositionFactor);
    // mPivotEncoder.setVelocityConversionFactor(kPivotEncoderVelocityFactor);

    // mPivotMotor.burnFlash();

    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    // double intake_pivot_voltage = 0.0;
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
    EJECT2,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // checkAutoTasks();

    // Pivot control
    double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);
    // double pivot_angle = pivotTargetToAngle(PivotTarget.GROUND);

    // m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getPivotAngleDegrees(),
    // pivot_angle);

    // mPivotPIDController.setReference(275-60, ControlType.kPosition);

    mPivotPIDController.setReference(pivot_angle, ControlType.kPosition);
    // mPivotPIDController.setReference(, ControlType.kPosition);

    // m_periodicIO.intake_pivot_voltage = mPivotPIDController.calculate(getPivotAngleDegrees(),
    // 60);

    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    // if (m_pivotEncoder.get() == 0.0) {
    //   m_periodicIO.intake_pivot_voltage = 0.0;
    // }

    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);

    writePeriodicOutputs();
    outputTelemetry();
  }

  // // @Override
  public void writePeriodicOutputs() {
    // mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);
    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  // @Override
  public void stop() {
    // m_periodicIO.intake_pivot_voltage = 0.0;
    m_periodicIO.intake_speed = 0.0;
  }

  // @Override
  public void outputTelemetry() {
    Logger.recordOutput("Intake/Intake State", m_periodicIO.intake_state.toString());
    Logger.recordOutput("Intake/Pivot State", m_periodicIO.pivot_target.toString());

    Logger.recordOutput("Pivot/Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    Logger.recordOutput("Pivot/Abs Enc (get)", mPivotEncoder.getPosition());

    Logger.recordOutput("Pivot/Angle", pivotTargetToAngle(m_periodicIO.pivot_target));
    Logger.recordOutput("Pivot/Abs Enc (getAbsolutePosition)", mPivotEncoder.getPosition());
    Logger.recordOutput("Pivot/Abs Enc (getPivotAngleDegrees)", getPivotAngleDegrees());
    Logger.recordOutput("Pivot/Setpoint", pivotTargetToAngle(m_periodicIO.pivot_target));

    // Logger.recordOutput("Pivot/Power", m_periodicIO.intake_pivot_voltage);
    Logger.recordOutput("Pivot/Current", mPivotMotor.getOutputCurrent());

    Logger.recordOutput("Limit Switch", getIntakeHasNote());
  }

  // @Override
  public void reset() {}

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
        // return 0.059415;
        // case SOURCE:
        //   return Constants.Intake.k_pivotAngleSource;
        // case AMP:
        //   return Constants.Intake.k_pivotAngleAmp;
        // return 0.3832;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
        // return 0.583;
      default:
        // "Safe" default
        // return 180;
        return Constants.Intake.k_pivotAngleStow;
    }
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        if (mcolorDetector.gotNote()) {
          return 0;
        }
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case EJECT2:
        return Constants.Intake.k_ejectSpeed + 0.25;
      case PULSE:
        // // Use the timer to pulse the intake on for a 1/16 second,
        // // then off for a 15/16 second
        // if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
        //   return Constants.Intake.k_intakeSpeed;
        // }
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
    double value = mPivotEncoder.getPosition() - Constants.Intake.k_pivotEncoderOffset + 0.5;

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

    // if (mcolorDetector.gotNote()) {
    //   // pulse();
    //   return true;
    // }
    return false;

    // return isPivotAtTarget(PivotTarget.GROUND);
  }

  // // Pivot helper functions
  public void goToGround() {
    System.out.println("awgertyu");
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  // public void goToSource() {
  //   System.out.println("wigKV;WFE;K");
  //   m_periodicIO.pivot_target = PivotTarget.SOURCE;
  //   m_periodicIO.intake_state = IntakeState.NONE;
  // }

  // public void goToAmp() {
  //   System.out.println("lhirerglio");

  //   m_periodicIO.pivot_target = PivotTarget.SOURCE;
  //   m_periodicIO.intake_state = IntakeState.NONE;
  // }

  public void goToStow() {
    System.out.println("wfek;wef");

    m_periodicIO.pivot_target = PivotTarget.STOW;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  // Intake helper functions
  public void intake() {
    System.out.println("in tasdke");
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    System.out.println("wertjweecfty6");
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    // m_periodicIO.intake_state = IntakeState.PULSE;
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

  // private void checkAutoTasks() {
  //   // If the intake is set to GROUND, and the intake has a note, and the pivot is
  //   // close to it's target
  //   // Stop the intake and go to the SOURCE position
  //   if (m_periodicIO.pivot_target == PivotTarget.GROUND
  //       && getIntakeHasNote()
  //       && isPivotAtTarget()) {
  //     m_periodicIO.pivot_target = PivotTarget.STOW;
  //     m_periodicIO.intake_state = IntakeState.PULSE;
  //   }
  // }

  private boolean isPivotAtTarget() {
    return Math.abs(mPivotEncoder.getPosition() - pivotTargetToAngle(m_periodicIO.pivot_target))
        < 0.001;
  }

  public boolean isPivotAtTarget(PivotTarget pivotTarget) {

    return Math.abs(mPivotEncoder.getPosition() - pivotTargetToAngle(pivotTarget)) < 0.09;
  }

  public PivotTarget getPivotTarget() {
    return m_periodicIO.pivot_target;
  }
}
