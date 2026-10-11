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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SPARK MAX + NEO position controller for a small gear / mechanism.
 *
 * <p>Holds a position setpoint under brake mode. Joystick jog slowly adjusts the
 * setpoint within software + SPARK soft limits; MAXMotion keeps motion gentle.
 *
 * <p>Units default to motor rotations (encoder conversion factor 1.0). Change
 * {@link #kMinPositionRotations} / {@link #kMaxPositionRotations} (and the matching
 * soft limits) to match your gear travel.
 */
public class Gear implements AutoCloseable {
  private static final int kCurrentLimitAmps = 10;

  // Travel range in motor rotations. Tune to the physical gear limits.
  private static final double kMinPositionRotations = 0.0;
  private static final double kMaxPositionRotations = 5.0;

  // Full stick deflection jogs this many motor rotations per second (very slow).
  private static final double kMaxJogRotationsPerSec = 0.25;
  private static final double kJoystickDeadband = 0.12;

  // MAXMotion profile: cruise ~15 RPM = 0.25 rev/s, mild accel.
  private static final double kCruiseVelocityRpm = 15.0;
  private static final double kMaxAccelRpmPerSec = 40.0;
  private static final double kAtTargetToleranceRotations = 0.05;

  // Light geared load for desktop sim.
  private static final double kSimMoiKgMetersSquared = 0.0003;
  private static final double kSimGearing = 1.0;
  private static final double kSimDtSeconds = 0.02;

  private final SparkMax motor;
  private final SparkClosedLoopController closedLoop;
  private final RelativeEncoder encoder;

  private final DCMotor simMotor = DCMotor.getNEO(1);
  private final SparkMaxSim motorSim;
  private final DCMotorSim mechanismSim;

  private double targetRotations = 0.0;
  private boolean holding = false;

  /**
   * @param canId SPARK MAX CAN device ID
   * @param inverted set true if motor direction is wrong
   */
  public Gear(int canId, boolean inverted) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    closedLoop = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake) // hold when not actively driven
        .smartCurrentLimit(kCurrentLimitAmps)
        .inverted(inverted);

    // Native NEO hall units: position = rotations, velocity = RPM
    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    // Soft limits mirror the software range (extra hardware safety net).
    config.softLimit
        .forwardSoftLimit(kMaxPositionRotations)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(kMinPositionRotations)
        .reverseSoftLimitEnabled(true);

    // Position loop (slot 0). MAXMotion does the slow profile; P holds.
    config.closedLoop
        .p(0.25)
        .i(0.0)
        .d(0.0)
        .outputRange(-1.0, 1.0);

    config.closedLoop.maxMotion
        .maxAcceleration(kMaxAccelRpmPerSec)
        .cruiseVelocity(kCruiseVelocityRpm)
        .allowedProfileError(0.1);

    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Start at the current mechanical position (treat boot pose as zero).
    encoder.setPosition(0.0);
    targetRotations = 0.0;
    motor.stopMotor();

    if (RobotBase.isSimulation()) {
      motorSim = new SparkMaxSim(motor, simMotor);
      mechanismSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(simMotor, kSimMoiKgMetersSquared, kSimGearing),
              simMotor);
    } else {
      motorSim = null;
      mechanismSim = null;
    }
  }

  /**
   * Advance REV + mechanism physics. Call from {@code TimedRobot.simulationPeriodic()}.
   *
   * @return simulated motor current draw (amps), or 0 when not simulating
   */
  public double simulationPeriodic() {
    if (motorSim == null || mechanismSim == null) {
      return 0.0;
    }

    double busVoltage = RoboRioSim.getVInVoltage();
    mechanismSim.setInputVoltage(motorSim.getAppliedOutput() * busVoltage);
    mechanismSim.update(kSimDtSeconds);
    motorSim.iterate(mechanismSim.getAngularVelocityRPM(), busVoltage, kSimDtSeconds);
    return holding ? motorSim.getMotorCurrent() : 0.0;
  }

  /**
   * Begin holding the current target (brake + closed-loop position).
   * Call once when entering teleop/test so the gear resists external loads.
   */
  public void enableHold() {
    holding = true;
    // Sync target to measured pose so we don't jump on enable.
    targetRotations = clamp(encoder.getPosition());
    applySetpoint();
  }

  /** Stop closed-loop output (brake idle mode still resists free spin). */
  public void disable() {
    holding = false;
    motor.stopMotor();
  }

  /**
   * Joystick jog helper. Call every teleop period with stick Y in [-1, 1]
   * (already sign-corrected so + = "up"/forward travel) and the loop dt.
   *
   * <p>Deadbanded stick keeps the existing setpoint (true hold). Deflection
   * slowly integrates the setpoint within the allowed range.
   *
   * @param stickY joystick axis after your preferred sign flip, [-1, 1]
   * @param dtSeconds period length (typically ~0.02)
   */
  public void jog(double stickY, double dtSeconds) {
    if (!holding) {
      return;
    }

    double axis = MathUtil.applyDeadband(stickY, kJoystickDeadband);
    if (axis != 0.0) {
      targetRotations =
          clamp(targetRotations + axis * kMaxJogRotationsPerSec * dtSeconds);
    }
    applySetpoint();
  }

  /**
   * Command an absolute position (clamped). Applies immediately if holding.
   *
   * @param rotations motor rotations (after conversion factor)
   */
  public void setPosition(double rotations) {
    targetRotations = clamp(rotations);
    if (holding) {
      applySetpoint();
    }
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getTargetPosition() {
    return targetRotations;
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

  public double getMinPosition() {
    return kMinPositionRotations;
  }

  public double getMaxPosition() {
    return kMaxPositionRotations;
  }

  public boolean isHolding() {
    return holding;
  }

  public boolean atTarget() {
    return Math.abs(encoder.getPosition() - targetRotations) <= kAtTargetToleranceRotations;
  }

  private void applySetpoint() {
    closedLoop.setSetpoint(
        targetRotations,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
  }

  private static double clamp(double rotations) {
    return MathUtil.clamp(rotations, kMinPositionRotations, kMaxPositionRotations);
  }

  /** Release the SPARK CAN device (needed between unit tests reusing an ID). */
  @Override
  public void close() {
    disable();
    motor.close();
  }

  public void reportTelemetry() {
    SmartDashboard.putNumber("Gear/PositionRot", getPosition());
    SmartDashboard.putNumber("Gear/TargetRot", getTargetPosition());
    SmartDashboard.putNumber("Gear/RPM", getVelocity());
    SmartDashboard.putNumber("Gear/Amps", getOutputCurrent());
    SmartDashboard.putNumber("Gear/AppliedOutput", getAppliedOutput());
    SmartDashboard.putNumber("Gear/BusVoltage", getBusVoltage());
    SmartDashboard.putBoolean("Gear/Holding", isHolding());
    SmartDashboard.putBoolean("Gear/AtTarget", atTarget());
    SmartDashboard.putNumber("Gear/MinRot", kMinPositionRotations);
    SmartDashboard.putNumber("Gear/MaxRot", kMaxPositionRotations);
  }
}
