package org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.pidUtils;


/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile.
 * Extends PIDController so that “goal → setpoint” is automatic (with zero velocity).
 */
public class ProfiledPIDController extends PIDController {
  private TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd,
   * using a default update period of 0.02 seconds.
   *
   * @param Kp The proportional coefficient. Must be >= 0.
   * @param Ki The integral coefficient. Must be >= 0.
   * @param Kd The derivative coefficient. Must be >= 0.
   * @param constraints Velocity and acceleration constraints for the trapezoid profile.
   * @throws IllegalArgumentException if Kp, Ki, Kd < 0
   */
  public ProfiledPIDController(
          double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
    super(Kp, Ki, Kd);
    m_constraints = constraints;
    m_profile = new TrapezoidProfile(m_constraints);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd,
   * and a custom update period.
   *
   * @param Kp The proportional coefficient. Must be >= 0.
   * @param Ki The integral coefficient. Must be >= 0.
   * @param Kd The derivative coefficient. Must be >= 0.
   * @param constraints Velocity and acceleration constraints for the trapezoid profile.
   * @param period The loop period (in seconds). Must be > 0.
   * @throws IllegalArgumentException if Kp, Ki, Kd < 0 or period <= 0
   */
  @SuppressWarnings("this-escape")
  public ProfiledPIDController(
          double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
    super(Kp, Ki, Kd, period);
    m_constraints = constraints;
    m_profile = new TrapezoidProfile(m_constraints);
  }

  /**
   * Sets the trapezoid‐profile goal position (velocity = 0 implicitly). Immediately calls
   * super.setSetpoint(goal).
   *
   * @param setpoint The desired position
   */
  @Override
  public void setSetpoint(double setpoint) {
    m_goal = new TrapezoidProfile.State(setpoint, 0.0);
    super.setSetpoint(setpoint);
  }

  /**
   * Returns true if both:
   *   1) the underlying PIDController is “at setpoint” (pos‐error < tolerance, vel‐error < tolerance), and
   *   2) the profile’s current setpoint state equals the goal state.
   */
  @Override
  public boolean atSetpoint() {
    return super.atSetpoint() && m_goal.equals(m_setpoint);
  }

  /**
   * Updates the velocity/acceleration constraints and rebuilds the internal profile.
   *
   * @param constraints New velocity & acceleration constraints.
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_constraints = constraints;
    m_profile = new TrapezoidProfile(m_constraints);
  }

  public void setMaxVelocity(Double maxVelocity) {
      setConstraints(new TrapezoidProfile.Constraints(maxVelocity, m_constraints.maxAcceleration));
  }

  public void setMaxAcceleration(double maxAcceleration) {
      setConstraints(new TrapezoidProfile.Constraints(m_constraints.maxVelocity, maxAcceleration));
  }

  /** Returns the current velocity/acceleration constraints. */
  public TrapezoidProfile.Constraints getConstraints() {
    return m_constraints;
  }

  /**
   * Overrides PIDController.calculate(measurement):
   *  1) Advances the trapezoid profile (from m_setpoint → m_goal) for one period
   *  2) Calls super.calculate(measurement, m_setpoint.position)
   *
   * @param measurement The current measured process variable.
   * @return the raw PID output (based on position‐only error to m_setpoint.position).
   */
  @Override
  public double calculate(double measurement) {
    if (isContinuousInputEnabled()) {
      // Get error which is the smallest distance between goal and measurement
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      double goalMinDistance =
              inputModulus(m_goal.position - measurement, -errorBound, errorBound);
      double setpointMinDistance =
              inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

      // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
      // may be outside the input range after this operation, but that's OK because the controller
      // will still go there and report an error of zero. In other words, the setpoint only needs to
      // be offset from the measurement by the input range modulus; they don't need to be equal.
      m_goal.position = goalMinDistance + measurement;
      m_setpoint.position = setpointMinDistance + measurement;
    }

    m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
    super.m_setpoint = m_setpoint.position;
    super.m_haveSetpoint = true;
    return super.calculate(measurement);
  }

  /**
   * Overload: “calculate” + a new goalPosition (double) in one shot.
   * Equivalent to setGoal(goal) and then calculate(measurement).
   */
  @Override
  public double calculate(double measurement, double setpoint) {
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  /**
   * Overload: “calculate” + a new goal state + new constraints in one shot.
   */
  //TODO: add calculate with custom constraints that doesn't change the constraint forever but just for this movement
  public double calculate(
          double measurement,
          double setpoint,
          TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return calculate(measurement, setpoint);
  }

  /**
   * Returns the current setpoint State of the trapezoid profile (position, velocity).
   * Note: this is *not* the same as PIDController.getSetpoint(). PIDController.getSetpoint()
   * returns only the position‐only setpoint that was last passed. Here, we give you both pos+vel.
   */
  public TrapezoidProfile.State getCurrentSetpointState() {
    return m_setpoint;
  }

  public TrapezoidProfile.State getGoalState() {
    return m_goal;
  }

  /**
   * Resets both:
   *   1) the internal trapezoid profile (so m_setpoint = measuredState)
   *   2) the PID integrator and previous error (via super.reset())
   *
   * @param measuredState The current measured position/velocity.
   */
  public void reset(TrapezoidProfile.State measuredState) {
    super.reset();                 // clears PID’s error, integral, etc.
    m_setpoint = new TrapezoidProfile.State(
            measuredState.position,
            measuredState.velocity
    );
  }

  /**
   * Overload: Reset using (position, velocity) in two doubles.
   */
  public void reset(double measuredPosition, double measuredVelocity) {
    reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
  }

  /**
   * Overload: Reset using only position (velocity assumed zero).
   */
  public void reset(double measuredPosition) {
    reset(measuredPosition, 0.0);
  }
}
