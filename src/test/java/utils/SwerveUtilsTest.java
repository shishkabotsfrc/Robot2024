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
    // Test no step
    assertEquals(1., SwerveUtils.StepTowards(1, 1, 1.), DELTA);
    assertEquals(1., SwerveUtils.StepTowards(1, 3, 0.), DELTA);
  }

  @Test
  void StepTowardsCircularTest() {
    // Test stepping forward
    assertEquals(Math.PI + 1., SwerveUtils.StepTowardsCircular(Math.PI, Math.PI + 1., 10.), DELTA);
    assertEquals(Math.PI + 1., SwerveUtils.StepTowardsCircular(Math.PI, Math.PI + 2., 1.), DELTA);
    // Test stepping backward
    assertEquals(Math.PI - 1., SwerveUtils.StepTowardsCircular(Math.PI, Math.PI - 1., 10.), DELTA);
    assertEquals(Math.PI - 1., SwerveUtils.StepTowardsCircular(Math.PI, Math.PI - 2., 1.), DELTA);
    // Test no step
    assertEquals(Math.PI, SwerveUtils.StepTowardsCircular(Math.PI, Math.PI, 10.), DELTA);
    assertEquals(Math.PI, SwerveUtils.StepTowardsCircular(Math.PI, Math.PI + 1., 0.), DELTA);
    // Test wraparound
    assertEquals(0.5, SwerveUtils.StepTowardsCircular(2. * Math.PI - 0.5, 0.5, 10.), DELTA);
    assertEquals(0.5, SwerveUtils.StepTowardsCircular(2. * Math.PI - 0.5, 1.0, 1.), DELTA);
  }

  @Test
  void AngleDifferenceTest() {
    // Simple delta
    assertEquals(Math.PI / 2., SwerveUtils.AngleDifference(Math.PI, Math.PI / 2.), DELTA);
    assertEquals(Math.PI / 2., SwerveUtils.AngleDifference(Math.PI / 2., Math.PI), DELTA);
    // Delta across 0
    assertEquals(Math.PI, SwerveUtils.AngleDifference(Math.PI / 2., -Math.PI / 2.), DELTA);
    assertEquals(Math.PI, SwerveUtils.AngleDifference(Math.PI / 2., 1.5 * Math.PI), DELTA);
    assertEquals(Math.PI, SwerveUtils.AngleDifference(1.5 * Math.PI, Math.PI / 2.), DELTA);
  }

  @Test
  void WrapAngleTest() {
    assertEquals(0., SwerveUtils.WrapAngle(0.), DELTA);
    assertEquals(0., SwerveUtils.WrapAngle(2. * Math.PI), DELTA);
    assertEquals(Math.PI, SwerveUtils.WrapAngle(Math.PI), DELTA);
    assertEquals(Math.PI, SwerveUtils.WrapAngle(3 * Math.PI), DELTA);
    assertEquals(Math.PI, SwerveUtils.WrapAngle(-Math.PI), DELTA);
    assertEquals(Math.PI, SwerveUtils.WrapAngle(-3 * Math.PI), DELTA);
  }

  @Test
  void WrapAngleDegreesTest() {
    assertEquals(0., SwerveUtils.WrapAngleDegrees(0.), DELTA);
    assertEquals(0., SwerveUtils.WrapAngleDegrees(360.), DELTA);
    assertEquals(180., SwerveUtils.WrapAngleDegrees(180.), DELTA);
    assertEquals(180., SwerveUtils.WrapAngleDegrees(180. + 360.), DELTA);
    assertEquals(180., SwerveUtils.WrapAngleDegrees(-180.), DELTA);
    assertEquals(180., SwerveUtils.WrapAngleDegrees(-180. - 360.), DELTA);
  }
}
