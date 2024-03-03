package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.CurrentTime;
import org.littletonrobotics.junction.Logger;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to detect pre-configured
 * colors.
 */
public class detectColorCommand extends Command {

  /*----------------------------------------------------------------------------*/
  /* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
  /* Open Source Software - may be modified and shared by FRC teams. The code   */
  /* must be accompanied by the FIRST BSD license file in the root directory of */
  /* the project.                                                               */
  /*----------------------------------------------------------------------------*/

  /** Change the I2C port below to match the connection of your color sensor */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter. The device will be
   * automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can be calibrated
   * ahead of time or during operation.
   *
   * <p>This object uses a simple euclidian distance to estimate the closest match with given
   * confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these are here as a basic
   * example.
   */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);

  private final Color kOrangeTarget =
      new Color(140.6965942 / 255, 92.6517334 / 255, 21.74459839 / 255);
  private String colorString = null;
  private double prevTime;

  @Override
  public void initialize() {
    m_colorMatcher.addColorMatch(kBlueTarget);

    m_colorMatcher.addColorMatch(kOrangeTarget);
  }

  public boolean gotNote() {

    if (colorString.equals("Orange") && timeElapsed(prevTime) < 1000) {
      return true;
    }
    return false;
  }

  private double timeElapsed(double prevTime) {
    return (CurrentTime.millis() - prevTime);
  }

  @Override
  public void execute() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be useful if
     * outputting the color to an RGB LED or similar. To read the raw color, use GetRawColor().
     *
     * <p>The color sensor works best when within a few inches from an object in well lit conditions
     * (the built in LED is a big help here!). The farther an object is the more light from the
     * surroundings will bleed into the measurements and make it difficult to accurately determine
     * its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /** Run the color match algorithm on our detected color */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    System.out.println(detectedColor.red + " " + detectedColor.green + " " + detectedColor.blue);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
      // System.out.println("Blue Found!" + match.confidence);
      prevTime = CurrentTime.millis();

    } else if (match.color == kOrangeTarget) {
      colorString = "Orange";
      // System.out.println("Orange found!" + match.confidence);
      prevTime = CurrentTime.millis();

    } else {
      colorString = "Unknown";
      System.out.println("unknown");
    }

    /** Open Smart Dashboard or Shuffleboard to see the color detected by the sensor. */
    Logger.recordOutput("Red", detectedColor.red);
    Logger.recordOutput("Green", detectedColor.green);
    Logger.recordOutput("Blue", detectedColor.blue);
    Logger.recordOutput("Confidence", match.confidence);
    Logger.recordOutput("Detected Color", colorString);
  }
}
