package frc.utils;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;

public class StepResponseAnalyzer {

  private final int BUFFER_LENGTH = 5;

  private double m_start;
  private double m_target;
  private double m_delta;
  private double m_maxValue;
  private double m_finalValue;

  private boolean m_unstable;
  private boolean m_settled;
  private double m_settlingTime;

  private Deque<Double> m_valueBuffer = new ArrayDeque<Double>();
  private Deque<Double> m_timeBuffer = new ArrayDeque<Double>();

  /**
   * Analyzer to calculate the response properties to a step function
   *
   * <p>Initialize this object and then use the @link{update} method to update the values one sample
   * at a time.
   *
   * @param start The start value of the step response
   * @param target The target value of the step respons
   * @param delta The delta threshold to use for the settling time
   */
  public StepResponseAnalyzer(double start, double target, double delta) {
    m_start = start;
    m_target = target;
    m_delta = delta;
    m_unstable = false;
    m_settled = false;
    m_settlingTime = -1.;
    m_finalValue = -1.;
    m_maxValue = start;
  }

  /**
   * Updates the analyzer with a new sample point
   *
   * @param time Current time in ms
   * @param value Current value
   */
  public void update(double time, double value) {
    // Update the max value
    m_maxValue = Math.max(m_maxValue, value);

    // Check for instability
    if (m_valueBuffer.size() == BUFFER_LENGTH && (value > 2 * m_target || value < m_start)) {
      m_unstable = true;
    }

    // Update the sliding buffer for the settling value and time
    m_valueBuffer.addLast(value);
    m_timeBuffer.addLast(time);
    if (m_valueBuffer.size() > BUFFER_LENGTH) {
      m_valueBuffer.removeFirst();
      m_timeBuffer.removeFirst();
    }
    if ((m_valueBuffer.size() < BUFFER_LENGTH) || m_settled) {
      return;
    }
    double min = m_valueBuffer.getFirst();
    double max = min;
    Iterator<Double> iter = m_valueBuffer.iterator();
    while (iter.hasNext()) {
      double x = iter.next();
      min = Math.min(min, x);
      max = Math.max(max, x);
    }
    if ((max - min) < m_delta && !m_settled) {
      m_settled = true;
      m_settlingTime = m_timeBuffer.getFirst();
      m_finalValue = m_valueBuffer.getFirst();
    }
  }

  public boolean isUnstable() {
    return m_unstable;
  }

  public boolean isSettled() {
    return m_settled;
  }

  public double settlingTime() {
    return m_settlingTime;
  }

  public double overshoot() {
    if (!m_settled) {
      return -1.;
    }
    if (m_maxValue <= m_finalValue + m_delta) {
      return 0;
    }
    return m_maxValue - m_finalValue;
  }

  public double finalValue() {
    return m_finalValue;
  }

  public double maxValue() {
    return m_maxValue;
  }
}
