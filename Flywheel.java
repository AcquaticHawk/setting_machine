import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SPARK MAX + NEO flywheel velocity controller.
 *
 * <p>Reach/hold a velocity setpoint under a smart current limit. Velocity may be
 * changed while running; MAXMotion ramps under accel + current limits.
 */
public class Flywheel {
  private static final int kCurrentLimitAmps = 2;
  private static final double kDefaultTargetRpm = 4690.0;
  private static final double kAtTargetToleranceRpm = 100.0;

  // NEO free speed ~5820 RPM @ 12 V. Used for feedforward guess + MAXMotion ceiling.
  private static final double kNeoFreeSpeedRpm = 5820.0;

  // Conservative accel/decel (RPM per second). Lower => gentler => less current spike.
  private static final double kMaxAccelRpmPerSec = 1000.0;

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoop;
  private final RelativeEncoder encoder;

  private double targetRpm = kDefaultTargetRpm;
  private boolean running = false;

  /**
   * @param canId SPARK MAX CAN device ID
   * @param inverted set true if motor direction is wrong
   */
  public Flywheel(int canId, boolean inverted) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    closedLoop = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kCoast) // flywheel coasts when disabled
        .smartCurrentLimit(kCurrentLimitAmps)
        .inverted(inverted);

    // Native NEO hall units: position = rotations, velocity = RPM
    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    // Velocity loop (slot 0). Gains are usually very small.
    config.closedLoop
        .p(0.00015)
        .i(0.0)
        .d(0.0)
        .velocityFF(1.0 / kNeoFreeSpeedRpm) // rough duty-cycle-per-RPM at 12 V
        .outputRange(-1.0, 1.0);

    // MAXMotion velocity: ramp to live setpoint under accel + current limits, then hold.
    config.closedLoop.maxMotion
        .maxAcceleration(kMaxAccelRpmPerSec)
        .maxVelocity(kNeoFreeSpeedRpm)
        .allowedClosedLoopError(50.0);

    // Persist so the current limit survives power cycles.
    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** Start (or restart) toward the current target RPM and hold. */
  public void enable() {
    running = true;
    applySetpoint();
  }

  /**
   * Change target velocity while stopped or while running.
   * If running, MAXMotion ramps to the new RPM under the current / accel limits.
   *
   * @param rpm target speed in RPM (0 allowed; negative reverses if desired)
   */
  public void setVelocity(double rpm) {
    targetRpm = rpm;
    if (running) {
      applySetpoint();
    }
  }

  /** Set velocity and ensure the controller is running. */
  public void setVelocityAndRun(double rpm) {
    targetRpm = rpm;
    running = true;
    applySetpoint();
  }

  /** Stop commanding the motor (coasts with IdleMode.kCoast). */
  public void disable() {
    running = false;
    motor.stopMotor();
  }

  public double getTargetVelocity() {
    return targetRpm;
  }

  public double getDefaultTargetRpm() {
    return kDefaultTargetRpm;
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }

  public boolean isRunning() {
    return running;
  }

  /** True when measured speed is within {@link #kAtTargetToleranceRpm} of target. */
  public boolean atTarget() {
    return atTarget(kAtTargetToleranceRpm);
  }

  public boolean atTarget(double toleranceRpm) {
    return Math.abs(encoder.getVelocity() - targetRpm) <= toleranceRpm;
  }

  private void applySetpoint() {
    closedLoop.setSetpoint(
        targetRpm,
        ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
  }

  public void reportTelemetry() {
    SmartDashboard.putNumber("Flywheel/RPM", getVelocity());
    SmartDashboard.putNumber("Flywheel/TargetRPM", getTargetVelocity());
    SmartDashboard.putNumber("Flywheel/Amps", getOutputCurrent());
    SmartDashboard.putNumber("Flywheel/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("Flywheel/BusVoltage", motor.getBusVoltage());
    SmartDashboard.putBoolean("Flywheel/Running", isRunning());
    SmartDashboard.putBoolean("Flywheel/AtTarget", atTarget());
  }
}
