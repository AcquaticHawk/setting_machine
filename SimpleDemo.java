// Simple teleop demo: flywheel + intake SPARK MAX / NEO pair
//
// Sequence (Teleop, Xbox port 0):
//   1. Start with all motors off
//   2. A - spin up flywheel to target RPM
//   3. When target is reached - controller rumble "beep" (ready signal)
//   4. X - start intake (only allowed after ready signal)
//   5. Motors keep running until B - stop everything (always, any state)
//
// Requires a WPILib + REVLib robot project deployed to a roboRIO.
// Hardware:
//   - Flywheel NEO  -> SPARK MAX CAN ID 1 (default)
//   - Intake NEO    -> SPARK MAX CAN ID 2 (default)
//   - Xbox controller on port 0
//
// Drop these files into src/main/java/ (adjust package if your project uses one).
// If your project already has Main.java / Robot.java, either:
//   - point Main at SimpleDemo::new, or
//   - copy Flywheel / Intake into your subsystems and delete Main below.

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Teleop-only demo: A spins up flywheel, ready beep, X starts intake, B kills all.
 */
public class SimpleDemo extends TimedRobot {
  // --- Hardware ---
  private static final int kFlywheelCanId = 1; // change to your flywheel SPARK MAX CAN ID
  private static final int kIntakeCanId = 2; // change to your intake SPARK MAX CAN ID
  private static final int kControllerPort = 0;

  private static final double kBeepDurationSec = 0.35;

  private final Flywheel flywheel = new Flywheel(kFlywheelCanId, false);
  private final Intake intake = new Intake(kIntakeCanId, false);
  private final XboxController driver = new XboxController(kControllerPort);
  private final Timer beepTimer = new Timer();

  /** True after flywheel first reaches target following an A press; gates X. */
  private boolean intakeAllowed = false;
  /** Ensures the ready beep fires only once per spin-up. */
  private boolean readyBeepPlayed = false;

  // Edge detection so held buttons don't repeat every 20 ms
  private boolean prevA;
  private boolean prevB;
  private boolean prevX;

  // ---------------------------------------------------------------------------
  // Robot lifecycle
  // ---------------------------------------------------------------------------

  @Override
  public void robotInit() {
    stopAll();
    reportTelemetry();
  }

  @Override
  public void robotPeriodic() {
    updateBeep();
    // Detect target reached while spinning up (also covers cases where A was
    // pressed and velocity arrives mid-period).
    if (flywheel.isRunning() && !readyBeepPlayed && flywheel.atTarget()) {
      playReadyBeep();
      intakeAllowed = true;
      readyBeepPlayed = true;
    }
    reportTelemetry();
  }

  @Override
  public void disabledInit() {
    stopAll();
  }

  @Override
  public void autonomousInit() {
    // No autonomous routine — motors stay off.
    stopAll();
  }

  @Override
  public void autonomousPeriodic() {
    // Intentionally empty.
  }

  @Override
  public void teleopInit() {
    // System starts with nothing running.
    stopAll();
  }

  @Override
  public void teleopPeriodic() {
    boolean a = driver.getAButton();
    boolean b = driver.getBButton();
    boolean x = driver.getXButton();

    // B always turns off all motors, regardless of state.
    if (b && !prevB) {
      stopAll();
    } else {
      // A starts flywheel spin-up. Intake stays gated until target is reached.
      if (a && !prevA) {
        intake.disable();
        intakeAllowed = false;
        readyBeepPlayed = false;
        flywheel.setVelocityAndRun(flywheel.getDefaultTargetRpm());
      }

      // X starts intake only after the ready beep (target reached).
      if (x && !prevX) {
        if (intakeAllowed) {
          intake.enable();
        }
      }
    }

    prevA = a;
    prevB = b;
    prevX = x;
  }

  @Override
  public void testInit() {
    stopAll();
  }

  @Override
  public void testPeriodic() {
    // Same controls as teleop for bench testing.
    teleopPeriodic();
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /** Stop flywheel + intake and clear ready/intake gate state. */
  private void stopAll() {
    flywheel.disable();
    intake.disable();
    intakeAllowed = false;
    readyBeepPlayed = false;
    stopBeep();
  }

  /** Short controller rumble used as the "at target" beep. */
  private void playReadyBeep() {
    beepTimer.reset();
    beepTimer.start();
    driver.setRumble(RumbleType.kBothRumble, 1.0);
  }

  private void updateBeep() {
    if (beepTimer.isRunning() && beepTimer.hasElapsed(kBeepDurationSec)) {
      stopBeep();
    }
  }

  private void stopBeep() {
    driver.setRumble(RumbleType.kBothRumble, 0.0);
    beepTimer.stop();
  }

  private void reportTelemetry() {
    flywheel.reportTelemetry();
    intake.reportTelemetry();
    SmartDashboard.putBoolean("Demo/IntakeAllowed", intakeAllowed);
    SmartDashboard.putBoolean("Demo/ReadyBeepPlayed", readyBeepPlayed);
  }
}

/** WPILib entry point. Remove if your project already provides Main. */
class Main {
  private Main() {}

  public static void main(String[] args) {
    RobotBase.startRobot(SimpleDemo::new);
  }
}
