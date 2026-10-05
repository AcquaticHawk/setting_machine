package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Desktop-sim checks for 10 A smart limit: reach default target and hold it.
 */
class FlywheelSimTest {
  private static final double kDtSeconds = 0.02;
  // 10 A + 500 RPM/s profile + sim MOI → allow a longer spool.
  private static final double kSpinUpTimeoutSeconds = 30.0;
  private static final double kHoldSeconds = 5.0;

  private Flywheel flywheel;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    RoboRioSim.resetData();
    RoboRioSim.setVInVoltage(12.0);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    if (flywheel != null) {
      flywheel.close();
      flywheel = null;
    }
  }

  @Test
  void reachesAndHoldsDefaultTargetRpmAt10A() {
    // Distinct CAN ID from other tests — close() is not always enough for REV sim.
    flywheel = new Flywheel(3, false);
    flywheel.setVelocityAndRun(flywheel.getDefaultTargetRpm());

    final int spinUpSteps = (int) Math.ceil(kSpinUpTimeoutSeconds / kDtSeconds);
    double rpm = 0.0;
    boolean reached = false;
    double peakAmps = 0.0;

    for (int i = 0; i < spinUpSteps; i++) {
      stepSim();
      rpm = flywheel.getVelocity();
      peakAmps = Math.max(peakAmps, Math.abs(flywheel.getOutputCurrent()));
      if (flywheel.atTarget()) {
        reached = true;
        break;
      }
    }

    assertTrue(
        reached,
        String.format(
            "Did not reach target within %.1fs (rpm=%.1f, target=%.1f, applied=%.3f, bus=%.2fV, "
                + "amps=%.2f, peakAmps=%.2f, isSim=%s)",
            kSpinUpTimeoutSeconds,
            rpm,
            flywheel.getDefaultTargetRpm(),
            flywheel.getAppliedOutput(),
            flywheel.getBusVoltage(),
            flywheel.getOutputCurrent(),
            peakAmps,
            RobotBase.isSimulation()));

    final int holdSteps = (int) Math.ceil(kHoldSeconds / kDtSeconds);
    double minRpm = Double.POSITIVE_INFINITY;
    double maxRpm = Double.NEGATIVE_INFINITY;
    double maxHoldAmps = 0.0;

    for (int i = 0; i < holdSteps; i++) {
      stepSim();
      rpm = flywheel.getVelocity();
      minRpm = Math.min(minRpm, rpm);
      maxRpm = Math.max(maxRpm, rpm);
      maxHoldAmps = Math.max(maxHoldAmps, Math.abs(flywheel.getOutputCurrent()));
    }

    assertTrue(
        flywheel.atTarget(),
        String.format(
            "Lost target while holding (rpm=%.1f, min=%.1f, max=%.1f, applied=%.3f, bus=%.2fV, "
                + "maxHoldAmps=%.2f)",
            flywheel.getVelocity(),
            minRpm,
            maxRpm,
            flywheel.getAppliedOutput(),
            flywheel.getBusVoltage(),
            maxHoldAmps));

    // Sanity: hold current should stay well under the 10 A spin-up budget when unloaded.
    assertTrue(
        maxHoldAmps < 10.0,
        String.format("Hold current too high for unloaded sim (maxHoldAmps=%.2f)", maxHoldAmps));
  }

  private void stepSim() {
    RoboRioSim.setVInVoltage(12.0);
    flywheel.simulationPeriodic();
    DriverStationSim.notifyNewData();
  }

  @Test
  void coastsSlowlyWhenDisabled() {
    flywheel = new Flywheel(1, false);
    flywheel.setVelocityAndRun(flywheel.getDefaultTargetRpm());

    final int spinUpSteps = (int) Math.ceil(kSpinUpTimeoutSeconds / kDtSeconds);
    boolean reached = false;
    for (int i = 0; i < spinUpSteps; i++) {
      stepSim();
      if (flywheel.atTarget()) {
        reached = true;
        break;
      }
    }
    assertTrue(reached, "precondition: reach target before coast check");

    double rpmAtDisable = flywheel.getVelocity();
    flywheel.disable();

    // 1 second of coast should only lose a few percent with good bearings — not slam to 0.
    final int coastSteps = (int) Math.ceil(1.0 / kDtSeconds);
    for (int i = 0; i < coastSteps; i++) {
      stepSim();
    }
    double rpmAfter1s = flywheel.getVelocity();
    double retained = rpmAfter1s / rpmAtDisable;

    assertTrue(
        retained > 0.90,
        String.format(
            "Coast too aggressive (dynamic brake?): rpm %.1f -> %.1f in 1s (retained=%.3f)",
            rpmAtDisable,
            rpmAfter1s,
            retained));
  }

}
