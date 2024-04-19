package frc.robot.subsystems.vision;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ColorSensor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  // See graph in #frc-programming
  private final Color kOrangeTarget = new Color(0.569, 0.349, 0.082);

  public void initialize() {
    m_colorMatcher.addColorMatch(kOrangeTarget);
  }

  /** If the color sensor detected a note stored in the intake */
  public boolean gotNote() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    // if (!match.color.equals(kOrangeTarget)) {
    //   System.err.println("[Color Sensor] Color matched = " + match.color);
    // }
    if (match.confidence < 0.5) {
      return false;
    }
    // Higher is closer (0-2047), spec sheet says range of 1-10cm
    if (m_colorSensor.getProximity() < 150) {
      return false;
    }
    return true;
  }

  /** Logs important stuff from sensor, used for tuning `gotNote()` */
  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    Logger.recordOutput("ColorSensor/Red", detectedColor.red);
    Logger.recordOutput("ColorSensor/Green", detectedColor.green);
    Logger.recordOutput("ColorSensor/Blue", detectedColor.blue);
    Logger.recordOutput("ColorSensor/Confidence", match.confidence);
    Logger.recordOutput("ColorSensor/DetectedColor", match.color.toHexString());
    Logger.recordOutput("ColorSensor/Hex", detectedColor.toHexString());
    Logger.recordOutput("ColorSensor/Proximity", m_colorSensor.getProximity());
    Logger.recordOutput("ColorSensor/IR", m_colorSensor.getIR());
  }
}
