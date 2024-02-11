package utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.utils.StepResponseAnalyzer;
import org.junit.jupiter.api.Test;

public class StepResponseAnalyzerTest {

  // In ms
  static final double maxTime = 2000.;
  static final double deltaTime = 20;
  static final double DELTA = 1e-2;

  // 1-exp(-10*x)
  // Damped response
  @Test
  void DampedResponseTest() {
    StepResponseAnalyzer analyzer = new StepResponseAnalyzer(0., 1., 0.01);
    for (double t = 0.; t <= maxTime; t += deltaTime) {
      double tSec = t / 1000.;
      double value = 1 - Math.exp(-10 * tSec);
      analyzer.update(t, value);
      if (analyzer.isSettled()) {
        break;
      }
      assertFalse(analyzer.isUnstable());
    }
    assertEquals(420.0, analyzer.settlingTime(), DELTA);
    assertEquals(0., analyzer.overshoot(), DELTA);
    assertEquals(0.985, analyzer.finalValue(), DELTA);
    assertEquals(0.993, analyzer.maxValue(), DELTA);
  }

  // 1-exp(-10*x) + sin(10*x)*exp(-4*x)
  @Test
  void UnderDampedResponseTest() {
    StepResponseAnalyzer analyzer = new StepResponseAnalyzer(0., 1., 0.01);
    for (double t = 0.; t <= maxTime; t += deltaTime) {
      double tSec = t / 1000.;
      double value = 1 - Math.exp(-10 * tSec) + Math.sin(10 * tSec) * Math.exp(-4 * tSec);
      analyzer.update(t, value);
      if (analyzer.isSettled()) {
        break;
      }
      assertFalse(analyzer.isUnstable());
    }
    assertTrue(analyzer.isSettled());
    assertEquals(700., analyzer.settlingTime(), DELTA);
    assertEquals(0.286, analyzer.overshoot(), DELTA);
    assertEquals(1.039, analyzer.finalValue(), DELTA);
    assertEquals(1.325, analyzer.maxValue(), DELTA);
  }

  // Unstable response
  @Test
  void UnstableResponsePositiveTest() {
    StepResponseAnalyzer analyzer = new StepResponseAnalyzer(0., 1., 0.01);
    for (double t = 0.; t <= maxTime; t += deltaTime) {
      double tSec = t / 1000.;
      double value = 2 * tSec;
      analyzer.update(t, value);
      if (analyzer.isUnstable()) {
        break;
      }
    }
    assertTrue(analyzer.isUnstable());
  }

  // Unstable response
  @Test
  void UnstableResponseNegativeTest() {
    StepResponseAnalyzer analyzer = new StepResponseAnalyzer(0., 1., 0.01);
    for (double t = 0.; t <= maxTime; t += deltaTime) {
      double tSec = t / 1000.;
      double value = -2 * tSec;
      analyzer.update(t, value);
      if (analyzer.isUnstable()) {
        break;
      }
    }
    assertTrue(analyzer.isUnstable());
  }
}
