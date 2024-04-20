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
  private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1500);
  public double limitedSpeed;

  public Shooter() {
    mLeftMotorSparkMax = new CANSparkMax(ShooterConstants.leftCanId, MotorType.kBrushless);
    mRightMotorSparkMax = new CANSparkMax(ShooterConstants.rightCanId, MotorType.kBrushless);

    mLeftMotorSparkMax.restoreFactoryDefaults();
    mRightMotorSparkMax.restoreFactoryDefaults();

    mLeftMotorPID = mLeftMotorSparkMax.getPIDController();
    mRightMotorPID = mRightMotorSparkMax.getPIDController();

    mLeftMotorPID.setP(0.00005);
    mRightMotorPID.setP(0.00005);
    mLeftMotorPID.setI(0);
    mRightMotorPID.setI(0);
    mLeftMotorPID.setD(0);
    mRightMotorPID.setD(0);

    mLeftMotorPID.setFF(0.0002);
    mRightMotorPID.setFF(0.0002);
    mLeftMotorPID.setOutputRange(0, 1);
    mRightMotorPID.setOutputRange(0, 1);

    mRightMotorEncoder = mRightMotorSparkMax.getEncoder();
    mLeftMotorEncoder = mLeftMotorSparkMax.getEncoder();

    mRightMotorSparkMax.setInverted(false);
    mLeftMotorSparkMax.setInverted(true);
  }

  public void setSpeed(double rpm) {
    System.out.println("[Shooter] target rpm = " + rpm);
    mMotorRPM = rpm;
  }

  public void stop() {
    mMotorRPM = 0.0;
  }

  public void periodic() {
    limitedSpeed = mSpeedLimiter.calculate(mMotorRPM);

    mLeftMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
    mRightMotorPID.setReference(limitedSpeed, ControlType.kVelocity);

    Logger.recordOutput("Shooter/targetRPM", mMotorRPM);
    Logger.recordOutput("Shooter/limitedRPM", limitedSpeed);
    Logger.recordOutput("Shooter/LeftVel", mLeftMotorEncoder.getVelocity());
    Logger.recordOutput("Shooter/RightVel", mRightMotorEncoder.getVelocity());
  }
}
