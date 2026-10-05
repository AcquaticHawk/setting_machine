package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
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
 * SPARK MAX + NEO intake wheels controller.
 *
 * <p>Simple on/off open-loop drive to feed game pieces into the flywheel.
 * Starts in the ON state; call {@link #enable()} / {@link #disable()} to switch.
 */
public class Intake {
  private static final int kCurrentLimitAmps = 5;
  private static final double kDefaultPower = 0.6;

  // Light rollers: small MOI is fine for open-loop sim feedback.
  private static final double kSimMoiKgMetersSquared = 0.0002;
  private static final double kSimGearing = 1.0;
  private static final double kSimDtSeconds = 0.02;

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  private final DCMotor simMotor = DCMotor.getNEO(1);
  private final SparkMaxSim motorSim;
  private final FlywheelSim mechanismSim;

  private double power = kDefaultPower;
  private boolean running = true; // start ON

  /**
   * @param canId SPARK MAX CAN device ID
   * @param inverted set true if motor direction is wrong
   */
  public Intake(int canId, boolean inverted) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    encoder = motor.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kBrake) // lock intake when disabled
        .smartCurrentLimit(kCurrentLimitAmps)
        .inverted(inverted);

    config.encoder
        .positionConversionFactor(1.0)
        .velocityConversionFactor(1.0);

    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Desired default is ON; output is applied when enable() runs (e.g. teleop/auto init).
    motor.stopMotor();

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
    // SPARK-limited current only — unlimited plant current collapses BatterySim Vin.
    return motorSim.getMotorCurrent();
  }

  /** Spin intake at the current power setting. */
  public void enable() {
    running = true;
    applyOutput();
  }

  /** Stop the intake wheels. */
  public void disable() {
    running = false;
    motor.stopMotor();
  }

  /** Toggle between on and off. */
  public void toggle() {
    if (running) {
      disable();
    } else {
      enable();
    }
  }

  /**
   * Change open-loop power. Applies immediately if currently running.
   *
   * @param power duty cycle in [-1.0, 1.0]
   */
  public void setPower(double power) {
    this.power = clamp(power, -1.0, 1.0);
    if (running) {
      applyOutput();
    }
  }

  /** Set power and ensure the intake is running. */
  public void setPowerAndRun(double power) {
    this.power = clamp(power, -1.0, 1.0);
    running = true;
    applyOutput();
  }

  public double getPower() {
    return power;
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

  private void applyOutput() {
    motor.set(power);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public void reportTelemetry() {
    SmartDashboard.putNumber("Intake/RPM", getVelocity());
    SmartDashboard.putNumber("Intake/Power", getPower());
    SmartDashboard.putNumber("Intake/Amps", getOutputCurrent());
    SmartDashboard.putNumber("Intake/AppliedOutput", motor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/BusVoltage", motor.getBusVoltage());
    SmartDashboard.putBoolean("Intake/Running", isRunning());
  }
}
