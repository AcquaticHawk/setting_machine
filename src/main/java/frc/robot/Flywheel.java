package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SPARK MAX + NEO flywheel velocity controller.
 *
 * <p>Reach/hold a velocity setpoint under a smart current limit. Velocity may be
 * changed while running; MAXMotion ramps under accel + current limits.
 */
public class Flywheel implements AutoCloseable {
  // Hardware budget: peak spin-up current. Hold at 4690 RPM unloaded is only a few amps.
  private static final int kCurrentLimitAmps = 14;
  // private static final double kDefaultTargetRpm = 4690.0;
  private static final double kDefaultTargetRpm = 2500.0;
  private static final double kAtTargetToleranceRpm = 100.0;

  // NEO free speed ~5820 RPM @ 12 V. Used for feedforward guess + MAXMotion ceiling.
  private static final double kNeoFreeSpeedRpm = 5820.0;
  // REVLib 2026 kV is volts per velocity unit (RPM here), not duty cycle per RPM.
  private static final double kNominalBatteryVolts = 12.0;
  private static final double kVelocityFfVoltsPerRpm = kNominalBatteryVolts / kNeoFreeSpeedRpm;

  // MAXMotion accel (RPM/s). ~500 → ~9 s profile to 4690; 10 A may stretch that further.
  private static final double kMaxAccelRpmPerSec = 200.0;

  // Sim plant MOI. Higher J = longer spool + longer coast (closer to a real wheel).
  private static final double kSimMoiKgMetersSquared = 0.002;
  private static final double kSimGearing = 1.0;
  private static final double kSimDtSeconds = 0.02;
  // Good ball bearings + light aero: velocity e-folds over tens of seconds when coasting.
  // (Not the electrical short you get from feeding the plant V=0.)
  private static final double kSimCoastTimeConstantSec = 40.0;

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoop;
  private final RelativeEncoder encoder;

  private final DCMotor simMotor = DCMotor.getNEO(1);
  private final SparkMaxSim motorSim;
  private final FlywheelSim mechanismSim;

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

    // Velocity loop (slot 0). P is small; kV does most of the work at speed.
    config.closedLoop
        .p(0.00015)
        .i(0.0)
        .d(0.0)
        .outputRange(-1.0, 1.0);

    // kV [V / RPM]: at free speed, FF ≈ nominal bus voltage → ~1.0 duty.
    config.closedLoop.feedForward.kV(kVelocityFfVoltsPerRpm);

    // MAXMotion velocity: ramp to live setpoint under accel + current limits, then hold.
    config.closedLoop.maxMotion
        .maxAcceleration(kMaxAccelRpmPerSec)
        .cruiseVelocity(kNeoFreeSpeedRpm)
        .allowedProfileError(50.0);

    // Persist so the current limit survives power cycles.
    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    if (RobotBase.isSimulation()) {
      motorSim = new SparkMaxSim(motor, simMotor);
      mechanismSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(simMotor, kSimMoiKgMetersSquared, kSimGearing),
              simMotor);
    } else {
      motorSim = null;
      mechanismSim = null;
    }
  }

  /**
   * Advance REV + mechanism physics. Call from {@code TimedRobot.simulationPeriodic()}.
   *
   * <p>Important: {@link LinearSystemId#createFlywheelSystem} assumes a voltage input. Feeding
   * {@code V=0} is <em>dynamic braking</em> (motor shorted through the H-bridge model), not
   * coast — decay time constant is ~J·R·Kv/Kt ≈ 0.1 s with a light wheel, which looks like huge
   * friction. With {@link IdleMode#kCoast} and the motor disabled we instead integrate open-circuit
   * coast (no electrical torque) plus mild bearing/aero drag.
   *
   * @return simulated motor current draw (amps), or 0 when not simulating
   */
  public double simulationPeriodic() {
    if (motorSim == null || mechanismSim == null) {
      return 0.0;
    }

    double busVoltage = RoboRioSim.getVInVoltage();

    if (!running) {
      // Open-circuit coast (IdleMode.kCoast): I ≈ 0 electrical torque.
      double w = mechanismSim.getAngularVelocityRadPerSec();
      w *= Math.exp(-kSimDtSeconds / kSimCoastTimeConstantSec);
      mechanismSim.setAngularVelocity(w);
    } else {
      mechanismSim.setInputVoltage(motorSim.getAppliedOutput() * busVoltage);
      mechanismSim.update(kSimDtSeconds);
    }

    motorSim.iterate(mechanismSim.getAngularVelocityRPM(), busVoltage, kSimDtSeconds);
    // Prefer SPARK-reported current (respects smart limit) over unlimited plant current.
    return running ? motorSim.getMotorCurrent() : 0.0;
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

  /** Release the SPARK CAN device (needed between unit tests reusing an ID). */
  @Override
  public void close() {
    disable();
    motor.close();
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

  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  public double getBusVoltage() {
    return motor.getBusVoltage();
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
