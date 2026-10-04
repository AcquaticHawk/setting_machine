// Complete runnable example: single SPARK MAX + NEO flywheel
//
// Goal:
//   - Reach/hold a velocity setpoint (default 4690 RPM)
//   - Enforce a 13 A smart current limit while changing speed
//   - Allow changing velocity while running
//
// Requires a WPILib + REVLib robot project deployed to a roboRIO.
// Hardware:
//   - NEO phase wires -> SPARK MAX
//   - NEO hall cable  -> SPARK MAX encoder port
//   - SPARK MAX on CAN bus (default CAN ID 1)
//   - Xbox controller on port 0 (optional; used in teleop)
//
// Drop this file into src/main/java/ (adjust package if your project uses one).
// If your project already has Main.java / Robot.java, either:
//   - point Main at FlywheelExample::new, or
//   - copy the flywheel methods into your subsystem and delete Main below.

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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Runnable TimedRobot demo with an onboard flywheel controller.
 *
 * Teleop controls (Xbox port 0):
 *   A                  - enable at current target (default 4690 RPM)
 *   B                  - disable / coast
 *   X                  - set 3000 RPM (applies immediately if running)
 *   Y                  - set 4690 RPM (applies immediately if running)
 *   Right bumper       - set 5200 RPM and run
 *   D-pad up / down    - nudge target by ±100 RPM (applies if running)
 *   Left bumper        - setVelocityAndRun(0)  (profile down to 0)
 */
public class FlywheelExample extends TimedRobot {
  // --- Hardware ---
  private static final int kCanId = 1; // change to your SPARK MAX CAN ID
  private static final int kControllerPort = 0;

  // --- Setpoints / limits ---
  private static final int kCurrentLimitAmps = 13;
  private static final double kDefaultTargetRpm = 4690.0;
  private static final double kAtTargetToleranceRpm = 100.0;

  // NEO free speed ~5820 RPM @ 12 V. Used for feedforward guess + MAXMotion ceiling.
  private static final double kNeoFreeSpeedRpm = 5820.0;

  // Conservative accel/decel (RPM per second). Lower => gentler => less current spike.
  // 1000 RPM/s => about 4.7 s from 0 to 4690.
  private static final double kMaxAccelRpmPerSec = 1000.0;

  private static final double kNudgeRpm = 100.0;

  private final SparkMax motor = new SparkMax(kCanId, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoop = motor.getClosedLoopController();
  private final RelativeEncoder encoder = motor.getEncoder();
  private final XboxController driver = new XboxController(kControllerPort);

  private double targetRpm = kDefaultTargetRpm;
  private boolean running = false;

  // Edge detection so held buttons don't spam setpoints every 20 ms
  private boolean prevA;
  private boolean prevB;
  private boolean prevX;
  private boolean prevY;
  private boolean prevRb;
  private boolean prevLb;
  private boolean prevPovUp;
  private boolean prevPovDown;

  public FlywheelExample() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .idleMode(IdleMode.kCoast) // flywheel coasts when disabled
        .smartCurrentLimit(kCurrentLimitAmps)
        .inverted(false); // set true if direction is wrong

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
    // maxVelocity is a profiler ceiling, not the command — keep near free speed.
    config.closedLoop.maxMotion
        .maxAcceleration(kMaxAccelRpmPerSec)
        .maxVelocity(kNeoFreeSpeedRpm)
        .allowedClosedLoopError(50.0);

    // Persist so the 13 A limit survives power cycles.
    motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // ---------------------------------------------------------------------------
  // Flywheel API
  // ---------------------------------------------------------------------------

  /** Start (or restart) toward the current target RPM and hold. */
  public void enable() {
    running = true;
    applySetpoint();
  }

  /**
   * Change target velocity while stopped or while running.
   * If running, MAXMotion ramps to the new RPM under the 13 A / accel limits.
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
    // If your REVLib build uses setReference(...), rename only this call.
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

  // ---------------------------------------------------------------------------
  // Robot lifecycle
  // ---------------------------------------------------------------------------

  @Override
  public void robotInit() {
    disable();
    reportTelemetry();
  }

  @Override
  public void robotPeriodic() {
    reportTelemetry();
  }

  @Override
  public void disabledInit() {
    disable();
  }

  @Override
  public void autonomousInit() {
    // Demo: spin to default 4690 RPM under 13 A + conservative accel.
    setVelocityAndRun(kDefaultTargetRpm);
  }

  @Override
  public void autonomousPeriodic() {
    // Hold whatever setpoint autonomousInit requested.
  }

  @Override
  public void teleopInit() {
    // Keep current state; driver chooses enable / speed below.
  }

  @Override
  public void teleopPeriodic() {
    boolean a = driver.getAButton();
    boolean b = driver.getBButton();
    boolean x = driver.getXButton();
    boolean y = driver.getYButton();
    boolean rb = driver.getRightBumperButton();
    boolean lb = driver.getLeftBumperButton();

    int pov = driver.getPOV();
    boolean povUp = pov == 0;
    boolean povDown = pov == 180;

    if (a && !prevA) {
      enable();
    }
    if (b && !prevB) {
      disable();
    }
    if (x && !prevX) {
      setVelocity(3000.0);
    }
    if (y && !prevY) {
      setVelocity(kDefaultTargetRpm);
    }
    if (rb && !prevRb) {
      setVelocityAndRun(5200.0);
    }
    if (lb && !prevLb) {
      setVelocityAndRun(0.0);
    }
    if (povUp && !prevPovUp) {
      setVelocity(targetRpm + kNudgeRpm);
    }
    if (povDown && !prevPovDown) {
      setVelocity(targetRpm - kNudgeRpm);
    }

    prevA = a;
    prevB = b;
    prevX = x;
    prevY = y;
    prevRb = rb;
    prevLb = lb;
    prevPovUp = povUp;
    prevPovDown = povDown;
  }

  @Override
  public void testInit() {
    disable();
  }

  @Override
  public void testPeriodic() {
    // Optional bench mode: hold Y to run default setpoint, release to stop.
    if (driver.getYButton()) {
      if (!isRunning() || getTargetVelocity() != kDefaultTargetRpm) {
        setVelocityAndRun(kDefaultTargetRpm);
      }
    } else if (isRunning()) {
      disable();
    }
  }
}

/** WPILib entry point. Remove if your project already provides Main. */
class Main {
  private Main() {}

  public static void main(String[] args) {
    RobotBase.startRobot(FlywheelExample::new);
  }
}
