package frc.robot.subsystems.vision;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class detectColor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  // See graph in #frc-programming
  private final Color kOrangeTarget = new Color(0.55, 0.36, 0.08);

  public void initialize() {
    m_colorMatcher.addColorMatch(kOrangeTarget);
  }

  public boolean gotNote() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color != kOrangeTarget) {
      System.err.println("Color sesnor matched: " + match.toString());
      return false;
    }
    // TODO: What numbers

    if (match.confidence < 0.8) {
      return false;
    }
    // if (m_colorSensor.getProximity() < 1337) {
    //   return false;
    // }

    return true;
  }

  /** Logs important stuff from sensor */
  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    Logger.recordOutput("ColorSensor/Red", detectedColor.red);
    Logger.recordOutput("ColorSensor/Green", detectedColor.green);
    Logger.recordOutput("ColorSensor/Blue", detectedColor.blue);
    Logger.recordOutput("ColorSensor/Confidence", match.confidence);
    Logger.recordOutput("ColorSensor/Proximity", m_colorSensor.getProximity());
  }
}
