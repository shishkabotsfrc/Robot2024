package utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.utils.SwerveUtils;
import org.junit.jupiter.api.Test;

public class SwerveUtilsTest {

  static final double DELTA = 1e-2;

  @Test
  void StepTowardsTest() {
    // Step target is less than the step size.
    assertEquals(3., SwerveUtils.StepTowards(1, 3, 10.), DELTA);
    // Step target is more than the step size.
    assertEquals(2., SwerveUtils.StepTowards(1, 3, 1.), DELTA);
  }
}
