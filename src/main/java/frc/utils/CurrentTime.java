package frc.utils;

import edu.wpi.first.util.WPIUtilJNI;

public class CurrentTime {
  public static double micros() {
    return WPIUtilJNI.now() * 1e-0;
  }

  public static double millis() {
    return WPIUtilJNI.now() * 1e-3;
  }

  public static double seconds() {
    return WPIUtilJNI.now() * 1e-6;
  }
}
