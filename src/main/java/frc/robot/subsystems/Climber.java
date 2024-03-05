package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private TalonFX mLeftMotor;
  private TalonFX mRightMotor;

  /*public Climber() {
    mLeftMotor = new TalonFX(ClimberConstants.leftCanId);
    mRightMotor = new TalonFX(ClimberConstants.rightCanId);
  }*/

  public void setBrakeMode(boolean brake) {
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    mLeftMotor.setNeutralMode(mode);
    mRightMotor.setNeutralMode(mode);
  }

  public void climb(double rpm) {
    mLeftMotor.set(rpm);
    mRightMotor.set(rpm);
  }

  public void periodic() {
    // Logger.recordOutput("Climber/LeftVel", mLeftMotor.getVelocity());
    // Logger.recordOutput("Climber/RightVel", mRightMotor.getVelocity());

    // Logger.recordOutput("Climber/LeftPos", mLeftMotor.getPosition());
    // Logger.recordOutput("Climber/RightPos", mRightMotor.getPosition());
  }
}
