package frc.robot.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Transition tests for the indexer, covering the half of M5's per-mechanism checkpoint that does
 * not need a robot.
 *
 * <p>The indexer's interesting states depend on what the hopper is doing to it, and plan section
 * 9.2 is explicit that game pieces are not simulated. {@link
 * IndexerIOSim#setSimulatedLoad(boolean)} and {@link IndexerIOSim#setSimulatedJam(boolean)} exist
 * so those paths can be exercised anyway. The single-piece cases this suite started with -- piece
 * detection, holding, the indexer's own shot counter -- were deleted with that model in M10b step
 * 8.
 */
public class IndexerFsmTest {
  private static final double LOOP_SEC = 0.02;

  private IndexerIOSim sim;
  private Indexer indexer;

  /** Builds an indexer over simulated IO with the HAL initialised and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    sim = new IndexerIOSim();
    indexer = new Indexer(sim);
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(indexer);
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /**
   * Advances the mechanism by a number of loops.
   *
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs periodic that many times and advances the simulated clock with it.
   */
  private void step(int loops) {
    for (int i = 0; i < loops; i++) {
      indexer.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** The machine boots idle, and stays there with nothing requested. */
  @Test
  void startsIdle() {
    step(loadDetectionLoops());
    assertEquals(Indexer.SystemState.IDLE, indexer.getSystemState());
  }

  /** Every SystemState is reachable, which is clause 2 of the M5 checkpoint. */
  @Test
  void everyStateIsReachable() {
    indexer.setWantedState(Indexer.WantedState.FEED);
    step(loadDetectionLoops());
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState());

    // UNJAMMING and JAMMED are only reachable against a jam, which is what the sim hook is for.
    sim.setSimulatedJam(true);
    boolean sawUnjamming = false;
    for (int i = 0; i < 400 && indexer.getSystemState() != Indexer.SystemState.JAMMED; i++) {
      step(1);
      sawUnjamming |= indexer.getSystemState() == Indexer.SystemState.UNJAMMING;
    }
    assertTrue(sawUnjamming, "a jam during FEED must reverse the wheel in UNJAMMING");
    assertEquals(
        Indexer.SystemState.JAMMED,
        indexer.getSystemState(),
        "a jam the pulses cannot clear must latch JAMMED");
    sim.setSimulatedJam(false);

    indexer.setWantedState(Indexer.WantedState.EJECT);
    step(5);
    assertEquals(Indexer.SystemState.EJECTING, indexer.getSystemState());

    indexer.setWantedState(Indexer.WantedState.CHARACTERIZE);
    step(5);
    assertEquals(Indexer.SystemState.CHARACTERIZING, indexer.getSystemState());

    indexer.setWantedState(Indexer.WantedState.IDLE);
    step(5);
    assertEquals(Indexer.SystemState.IDLE, indexer.getSystemState());
  }

  // ------------------------------------------------------------- M10b step 6: jams and load ----

  /** Loops for a load or jam to register: the spin-up settle window plus the longer debounce. */
  private static int loadDetectionLoops() {
    return (int)
            ((Constants.Indexer.kDetectionSettleSec.get()
                    + Math.max(
                        Constants.Indexer.kLoadedDebounceSec.get(),
                        Constants.Indexer.kJamDetectSec.get()))
                / LOOP_SEC)
        + 10;
  }

  /**
   * "Loaded" means a FEED pushing against something, and nothing else: it is false with no load,
   * false while idle with the load present, false while ejecting, and true only while feeding into
   * the load.
   *
   * <p>The not-feeding cases are the point. The old piece detector had to answer its question for a
   * stopped wheel and needed a whole replay mechanism to do it; this reading refuses to answer
   * there, and a version that leaked out of FEEDING would tell the superstructure "still moving
   * pieces" while nothing moves.
   */
  @Test
  void loadedMeansFeedingIntoALoadAndNothingElse() {
    indexer.setWantedState(Indexer.WantedState.FEED);
    step(loadDetectionLoops());
    assertFalse(indexer.isLoaded(), "an unloaded feed reported a load");

    sim.setSimulatedLoad(true);
    step(loadDetectionLoops());
    assertTrue(indexer.isLoaded(), "a feed into the hopper's load was not detected");
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState(), "a load is not a jam");

    for (Indexer.WantedState notFeeding :
        new Indexer.WantedState[] {Indexer.WantedState.IDLE, Indexer.WantedState.EJECT}) {
      indexer.setWantedState(notFeeding);
      for (int i = 0; i < 30; i++) {
        step(1);
        assertFalse(indexer.isLoaded(), "reported a load while " + indexer.getSystemState());
      }
    }
  }

  /**
   * A feed starting from rest is neither loaded nor jammed while the wheel spins up, however slowly
   * -- and the same trace, once past the settle window, does trip the jam detector.
   *
   * <p>Spin-up draws high current at low speed, which satisfies the load test AND the jam test at
   * once. The settle window is the only thing between that and a spurious unjam pulse at the start
   * of every burst, so the spin-up here deliberately looks exactly like a jam, for longer than the
   * jam debounce and shorter than the window.
   *
   * <p>Scripted, not simulated. The first version drove the simulated indexer, which with its
   * placeholder inertia reaches speed inside one 20 ms loop, so no transient ever showed up and the
   * case passed with the settle window deleted. A real indexer starting against a packed hopper is
   * not that quick. The positive control at the end proves the scripted stall is one the detector
   * can see at all.
   */
  @Test
  void aSlowSpinUpIsNeitherLoadedNorJammedButALaterStallIs() {
    int settleLoops = (int) (Constants.Indexer.kDetectionSettleSec.get() / LOOP_SEC);
    int jamDebounceLoops = (int) Math.ceil(Constants.Indexer.kJamDetectSec.get() / LOOP_SEC);
    int spinUpLoops = settleLoops - 3;
    assertTrue(
        spinUpLoops > jamDebounceLoops + 2,
        "the scripted spin-up must outlast the jam debounce, or this case tests nothing");

    StallScriptIO script = new StallScriptIO(spinUpLoops);
    CommandScheduler.getInstance().unregisterSubsystem(indexer);
    indexer = new Indexer(script);

    indexer.setWantedState(Indexer.WantedState.FEED);
    for (int i = 0; i < spinUpLoops + 30; i++) {
      step(1);
      assertFalse(indexer.isLoaded(), "the spin-up read as a load at loop " + i);
      assertEquals(
          Indexer.SystemState.FEEDING,
          indexer.getSystemState(),
          "the spin-up read as a jam at loop " + i);
    }

    // Positive control: the same stall, now that the window has passed, is a jam.
    script.stalled = true;
    boolean sawUnjam = false;
    for (int i = 0; i < 3 * jamDebounceLoops && !sawUnjam; i++) {
      step(1);
      sawUnjam = indexer.getSystemState() == Indexer.SystemState.UNJAMMING;
    }
    assertTrue(sawUnjam, "a stall past the settle window was not detected as a jam");
  }

  /**
   * An indexer IO that reports a scripted stall: for the first loops of any forward feed, and again
   * whenever {@link #stalled} is set, it reads as stopped against well over the jam current;
   * otherwise it reads as at speed and barely loaded.
   */
  private static final class StallScriptIO implements IndexerIO {
    private final int spinUpLoops;
    private double goalRpm = 0.0;
    private int loopsFeeding = 0;

    /** Set by the case to stall the wheel after the spin-up. */
    boolean stalled = false;

    /**
     * Builds a script whose spin-up looks stalled for a given number of loops.
     *
     * @param spinUpLoops loops, from the start of a forward feed, that read as stalled
     */
    StallScriptIO(int spinUpLoops) {
      this.spinUpLoops = spinUpLoops;
    }

    /**
     * Reports the scripted stall or the at-speed reading.
     *
     * @param inputs the struct to populate
     */
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
      inputs.connected = true;
      loopsFeeding = goalRpm > 0.0 ? loopsFeeding + 1 : 0;
      boolean stalledNow = goalRpm > 0.0 && (loopsFeeding <= spinUpLoops || stalled);
      inputs.velocityRpm = stalledNow ? 0.5 * Constants.Indexer.kJamVelocityRpm.get() : goalRpm;
      inputs.statorCurrentAmps = stalledNow ? 1.5 * Constants.Indexer.kJamCurrentAmps.get() : 2.0;
    }

    /**
     * Records the commanded speed.
     *
     * @param velocityRpm the speed, in RPM
     */
    @Override
    public void setVelocity(double velocityRpm) {
      goalRpm = velocityRpm;
    }

    /** Records a stop. */
    @Override
    public void stop() {
      goalRpm = 0.0;
    }
  }

  /**
   * A jam gets exactly {@code kMaxUnjamAttempts} reverse pulses, each lasting {@code kUnjamSec},
   * then the indexer stops trying, latches JAMMED with its alert, and stays latched while FEED is
   * still requested -- until the request changes.
   *
   * <p>Pulses are counted as entries into UNJAMMING, not read off the end state, because "it
   * retried forever" and "it retried three times" both end in a state that looks like a jam.
   */
  @Test
  void aJamIsRetriedABoundedNumberOfTimesThenLatches() {
    indexer.setWantedState(Indexer.WantedState.FEED);
    step(loadDetectionLoops());
    sim.setSimulatedJam(true);

    int pulses = 0;
    int longestPulseLoops = 0;
    int currentPulseLoops = 0;
    Indexer.SystemState previous = indexer.getSystemState();
    for (int i = 0; i < 400 && indexer.getSystemState() != Indexer.SystemState.JAMMED; i++) {
      step(1);
      Indexer.SystemState state = indexer.getSystemState();
      if (state == Indexer.SystemState.UNJAMMING) {
        if (previous != Indexer.SystemState.UNJAMMING) {
          pulses++;
          currentPulseLoops = 0;
        }
        currentPulseLoops++;
        longestPulseLoops = Math.max(longestPulseLoops, currentPulseLoops);
        assertEquals(
            Constants.Indexer.kUnjamRpm.get(),
            indexer.getGoalRpm(),
            "an unjam pulse must reverse the wheel");
      }
      previous = state;
    }

    assertEquals(Indexer.SystemState.JAMMED, indexer.getSystemState(), "never gave up on the jam");
    assertEquals(Constants.Indexer.kMaxUnjamAttempts, pulses, "wrong number of unjam pulses");
    assertEquals(
        Constants.Indexer.kUnjamSec.get() / LOOP_SEC,
        longestPulseLoops,
        1.0,
        "an unjam pulse did not last kUnjamSec");
    assertEquals(0.0, indexer.getGoalRpm(), "a jammed indexer must stop, not keep pushing");

    // Latched while FEED is still requested, even with the jam gone.
    sim.setSimulatedJam(false);
    step(50);
    assertEquals(
        Indexer.SystemState.JAMMED,
        indexer.getSystemState(),
        "the JAMMED latch released on its own while FEED was still requested");

    // A change of request releases it, and the next FEED starts with a full allowance.
    indexer.setWantedState(Indexer.WantedState.IDLE);
    step(3);
    indexer.setWantedState(Indexer.WantedState.FEED);
    step(loadDetectionLoops());
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState());

    // "Full allowance" is asserted, not assumed: jam it again, and the first response must be a
    // pulse, not an immediate JAMMED left over from the attempts the previous request spent.
    sim.setSimulatedJam(true);
    Indexer.SystemState firstResponse = Indexer.SystemState.FEEDING;
    for (int i = 0; i < 200 && firstResponse == Indexer.SystemState.FEEDING; i++) {
      step(1);
      firstResponse = indexer.getSystemState();
    }
    assertEquals(
        Indexer.SystemState.UNJAMMING,
        firstResponse,
        "a new FEED request inherited the spent unjam allowance of the last one");
  }

  /** A jam that one pulse clears goes back to feeding, and stays there. */
  @Test
  void aJamClearedByAPulseResumesFeeding() {
    indexer.setWantedState(Indexer.WantedState.FEED);
    step(loadDetectionLoops());
    sim.setSimulatedJam(true);
    for (int i = 0; i < 200 && indexer.getSystemState() != Indexer.SystemState.UNJAMMING; i++) {
      step(1);
    }
    assertEquals(Indexer.SystemState.UNJAMMING, indexer.getSystemState(), "the jam never fired");

    sim.setSimulatedJam(false);
    for (int i = 0; i < 150; i++) {
      step(1);
      assertTrue(
          indexer.getSystemState() == Indexer.SystemState.UNJAMMING
              || indexer.getSystemState() == Indexer.SystemState.FEEDING,
          "a cleared jam went to " + indexer.getSystemState());
    }
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState());
  }

  /** With hardware that never moves, the machine still transitions but detects nothing. */
  @Test
  void deadHardwareTransitionsButDetectsNothing() {
    CommandScheduler.getInstance().unregisterSubsystem(indexer);
    indexer = new Indexer(new IndexerIO() {});

    indexer.setWantedState(Indexer.WantedState.FEED);
    step(loadDetectionLoops());

    assertEquals(
        Indexer.SystemState.FEEDING,
        indexer.getSystemState(),
        "the state machine must still run when the mechanism cannot");
    assertFalse(indexer.isLoaded(), "a dead indexer must not report a load");
  }
}
