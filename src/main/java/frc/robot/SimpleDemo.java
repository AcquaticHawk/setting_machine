// Simple teleop demo: flywheel + intake + gear SPARK MAX / NEO
//
// Sequence (Teleop, Xbox port 0):
//   1. Start with flywheel/intake off; gear holds position
//   2. A - spin up flywheel to target RPM
//   3. When target is reached - controller rumble "beep" (ready signal)
//   4. X - start intake (only allowed after ready signal)
//   5. B - stop flywheel + intake (gear keeps holding)
//   6. Right stick Y - slowly jog gear within its position range (holds when centered)
//
// Requires a WPILib + REVLib robot project deployed to a roboRIO.
// Hardware:
//   - Flywheel NEO  -> SPARK MAX CAN ID 1 (default)
//   - Intake NEO    -> SPARK MAX CAN ID 2 (default)
//   - Gear NEO      -> SPARK MAX CAN ID 3 (default)
//   - Xbox controller on port 0
//
// Drop these files into src/main/java/ (adjust package if your project uses one).
// If your project already has Main.java / Robot.java, either:
//   - point Main at SimpleDemo::new, or
//   - copy Flywheel / Intake / Gear into your subsystems and delete Main below.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Teleop-only demo: A spins up flywheel, ready beep, X starts intake, B kills shooters,
 * right stick Y slowly jogs a position-holding gear.
 */
public class SimpleDemo extends TimedRobot {
  // --- Hardware ---
  private static final int kFlywheelCanId = 1; // change to your flywheel SPARK MAX CAN ID
  private static final int kIntakeCanId = 2; // change to your intake SPARK MAX CAN ID
  private static final int kGearCanId = 3; // change to your gear SPARK MAX CAN ID
  private static final int kControllerPort = 0;

  private static final double kBeepDurationSec = 0.35;
  /** Teleop loop period used to integrate joystick jog rate into gear position. */
  private static final double kTeleopDtSeconds = 0.02;

  private final Flywheel flywheel = new Flywheel(kFlywheelCanId, false);
  private final Intake intake = new Intake(kIntakeCanId, false);
  private final Gear gear = new Gear(kGearCanId, false);
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
  public void simulationPeriodic() {
    double flywheelAmps = flywheel.simulationPeriodic();
    double intakeAmps = intake.simulationPeriodic();
    double gearAmps = gear.simulationPeriodic();
    // Use |I| so sign quirks cannot inflate load; floor Vin so a transient spike
    // cannot permanently brown out the sim and pin the flywheel at ~500 RPM.
    double loaded =
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            Math.abs(flywheelAmps), Math.abs(intakeAmps), Math.abs(gearAmps));
    RoboRioSim.setVInVoltage(Math.max(10.0, loaded));
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
    // Shooters start off; gear begins holding its current pose.
    stopShooters();
    gear.enableHold();
  }

  @Override
  public void teleopPeriodic() {
    boolean a = driver.getAButton();
    boolean b = driver.getBButton();
    boolean x = driver.getXButton();

    // B always turns off flywheel + intake; gear keeps holding position.
    if (b && !prevB) {
      stopShooters();
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

    // Right stick Y: up jogs gear toward max, down toward min; centered holds.
    // WPILib reports stick up as negative, so negate for natural "up = forward".
    gear.jog(-driver.getRightY(), kTeleopDtSeconds);

    prevA = a;
    prevB = b;
    prevX = x;
  }

  @Override
  public void testInit() {
    // Match teleop so bench testing includes gear hold + jog.
    stopShooters();
    gear.enableHold();
  }

  @Override
  public void testPeriodic() {
    // Same controls as teleop for bench testing.
    teleopPeriodic();
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  /** Stop every actuator (used in disabled / robot init). */
  private void stopAll() {
    stopShooters();
    gear.disable();
  }

  /** Stop flywheel + intake and clear ready/intake gate state. Gear is unchanged. */
  private void stopShooters() {
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
    gear.reportTelemetry();
    SmartDashboard.putBoolean("Demo/IntakeAllowed", intakeAllowed);
    SmartDashboard.putBoolean("Demo/ReadyBeepPlayed", readyBeepPlayed);
  }
}
