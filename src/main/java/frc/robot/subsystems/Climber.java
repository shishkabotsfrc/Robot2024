package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private CANSparkMax mLeftMotorSparkMax;
  private CANSparkMax mRightMotorSparkMax;

  private RelativeEncoder mLeftMotorEncoder;
  private RelativeEncoder mRightMotorEncoder;

  private SparkMaxPIDController mLeftMotorPID;
  private SparkMaxPIDController mRightMotorPID;

  public Climber() {
    mLeftMotorSparkMax = new CANSparkMax(ClimberConstants.leftCanId, MotorType.kBrushless);
    mRightMotorSparkMax = new CANSparkMax(ClimberConstants.rightCanId, MotorType.kBrushless);

    mLeftMotorSparkMax.restoreFactoryDefaults();
    mRightMotorSparkMax.restoreFactoryDefaults();

    mLeftMotorPID = mLeftMotorSparkMax.getPIDController();
    mRightMotorPID = mRightMotorSparkMax.getPIDController();

    mRightMotorEncoder = mRightMotorSparkMax.getEncoder();
    mLeftMotorEncoder = mLeftMotorSparkMax.getEncoder();

    mRightMotorSparkMax.setInverted(true);
    mLeftMotorSparkMax.setInverted(true);
  }

  public void setBrakeMode(boolean brake) {
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
    mLeftMotorSparkMax.setIdleMode(mode);
    mRightMotorSparkMax.setIdleMode(mode);
  }

  public void climb(double rpm) {
    mLeftMotorPID.setReference(rpm, ControlType.kVelocity);
    mRightMotorPID.setReference(rpm, ControlType.kVelocity);
  }

  public void periodic() {
    Logger.recordOutput("Climber/LeftVel", mLeftMotorEncoder.getVelocity());
    Logger.recordOutput("Climber/RightVel", mRightMotorEncoder.getVelocity());

    Logger.recordOutput("Climber/LeftPos", mLeftMotorEncoder.getPosition());
    Logger.recordOutput("Climber/RightPos", mRightMotorEncoder.getPosition());
  }
}
