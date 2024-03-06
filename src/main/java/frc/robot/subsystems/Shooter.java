package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private CANSparkMax mLeftMotorSparkMax;
  private CANSparkMax mRightMotorSparkMax;

  private RelativeEncoder mLeftMotorEncoder;
  private RelativeEncoder mRightMotorEncoder;

  private SparkMaxPIDController mLeftMotorPID;
  private SparkMaxPIDController mRightMotorPID;

  private double mMotorRPM = 0;
  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(100);

  public Shooter() {
    mLeftMotorSparkMax = new CANSparkMax(ShooterConstants.leftCanId, MotorType.kBrushless);
    mRightMotorSparkMax = new CANSparkMax(ShooterConstants.rightCanId, MotorType.kBrushless);

    mLeftMotorSparkMax.restoreFactoryDefaults();
    mRightMotorSparkMax.restoreFactoryDefaults();

    mLeftMotorPID = mLeftMotorSparkMax.getPIDController();
    mLeftMotorPID.setP(0.2);
    mLeftMotorPID.setP(0.2);

    mRightMotorPID = mRightMotorSparkMax.getPIDController();

    mRightMotorEncoder = mRightMotorSparkMax.getEncoder();
    mLeftMotorEncoder = mLeftMotorSparkMax.getEncoder();

    mRightMotorSparkMax.setInverted(true);
    mLeftMotorSparkMax.setInverted(false);
  }

  public void setSpeed(double rpm) {
    System.out.println("set" + rpm);
    mMotorRPM = rpm;
  }

  public void stop() {
    mMotorRPM = 0.0;
  }

  public void periodic() {
    double limitedSpeed = mSpeedLimiter.calculate(mMotorRPM);
    mLeftMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightMotorPID.setReference(limitedSpeed, ControlType.kVelocity);

    Logger.recordOutput("Shooter/targetRPM", mMotorRPM);
    Logger.recordOutput("Shooter/LeftVel", mLeftMotorEncoder.getVelocity());
    Logger.recordOutput("Shooter/RightVel", mRightMotorEncoder.getVelocity());
  }
}
