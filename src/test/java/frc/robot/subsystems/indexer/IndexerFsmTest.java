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
 * <p>The indexer is the one M5 mechanism whose interesting states depend on a game piece, and plan
 * section 9.2 is explicit that game pieces are not simulated. {@link
 * IndexerIOSim#setSimulatedPiecePresent(boolean)} exists precisely so those paths can be exercised
 * anyway, and this suite is its first real consumer.
 */
public class IndexerFsmTest {
  private static final double LOOP_SEC = 0.02;

  /**
   * Loops needed for a piece to be detected, derived from the constants rather than guessed.
   *
   * <p>Detection has to clear two gates in series: the settling window that skips the spin-up
   * transient, and then the debouncer. A fixed guess sat right on that boundary and made this suite
   * fail intermittently, which is the worst possible failure mode for a test -- it reads as
   * flakiness and gets ignored. Deriving it means retuning either constant cannot silently
   * re-introduce that.
   *
   * @return loops to step before asserting on piece detection
   */
  private static int detectionLoops() {
    return (int)
            ((Constants.Indexer.kDetectionSettleSec.get()
                    + Constants.Indexer.kHasPieceDebounceSec.get())
                / LOOP_SEC)
        + 15;
  }

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

  /** Nothing is held at boot, and an empty indexer does not invent a piece. */
  @Test
  void startsIdleAndEmpty() {
    step(detectionLoops());
    assertEquals(Indexer.SystemState.IDLE, indexer.getSystemState());
    assertFalse(indexer.hasPiece(), "an empty indexer must not report a piece");
    assertEquals(0, indexer.getShotCount());
  }

  /** Every SystemState is reachable, which is clause 2 of the M5 checkpoint. */
  @Test
  void everyStateIsReachable() {
    indexer.setWantedState(Indexer.WantedState.INDEX);
    step(5);
    assertEquals(Indexer.SystemState.INDEXING, indexer.getSystemState());

    // HOLDING is only reachable with a piece aboard, which is what the sim hook is for.
    sim.setSimulatedPiecePresent(true);
    step(detectionLoops());
    assertEquals(
        Indexer.SystemState.HOLDING,
        indexer.getSystemState(),
        "a piece arriving during INDEX must promote the machine to HOLDING");

    indexer.setWantedState(Indexer.WantedState.FEED);
    step(5);
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState());

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

  /**
   * A piece fed out counts exactly one shot, and the count does not drift afterwards.
   *
   * <p>The drift half matters as much as the increment. The shot counter is armed by a latch rather
   * than by an instantaneous edge, so a latch that never clears would keep counting every loop the
   * indexer spent empty.
   */
  @Test
  void shotCountIncrementsExactlyOncePerPiece() {
    sim.setSimulatedPiecePresent(true);
    indexer.setWantedState(Indexer.WantedState.INDEX);
    step(detectionLoops());
    assertTrue(indexer.hasPiece(), "the injected piece was never detected");

    indexer.setWantedState(Indexer.WantedState.FEED);
    step(5);

    sim.setSimulatedPiecePresent(false);
    step(detectionLoops());
    assertEquals(1, indexer.getShotCount(), "feeding one piece out must count exactly one shot");

    step(100);
    assertEquals(1, indexer.getShotCount(), "the shot count drifted while the indexer sat empty");
  }

  /** Feeding an empty indexer counts nothing, however long it runs. */
  @Test
  void feedingAnEmptyIndexerCountsNothing() {
    indexer.setWantedState(Indexer.WantedState.FEED);
    step(150);
    assertEquals(0, indexer.getShotCount(), "an empty indexer counted a shot it never had");
  }

  /** The held piece survives HOLDING, where the wheel is deliberately stopped. */
  @Test
  void aHeldPieceIsNotLostWhileHolding() {
    sim.setSimulatedPiecePresent(true);
    indexer.setWantedState(Indexer.WantedState.INDEX);
    step(detectionLoops());

    indexer.setWantedState(Indexer.WantedState.HOLD);
    step(150);

    // The failure this guards is subtle and was fixed during implementation: detection reads the
    // load on a TURNING wheel, so a naive predicate reports "no piece" the moment HOLDING stops it,
    // the machine drops back to INDEXING, and the piece is ground at 50 Hz for the rest of a match.
    assertTrue(indexer.hasPiece(), "the held piece was lost once the wheel stopped");
    assertEquals(Indexer.SystemState.HOLDING, indexer.getSystemState());
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
   * false while held or idle with the load present, false while ejecting, and true only while
   * feeding into the load.
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
        new Indexer.WantedState[] {
          Indexer.WantedState.HOLD, Indexer.WantedState.IDLE, Indexer.WantedState.EJECT
        }) {
      indexer.setWantedState(notFeeding);
      for (int i = 0; i < 30; i++) {
        step(1);
        assertFalse(indexer.isLoaded(), "reported a load while " + indexer.getSystemState());
      }
    }
  }

  /**
   * A feed starting from rest is neither loaded nor jammed while the wheel spins up.
   *
   * <p>Spin-up draws high current at low speed, which satisfies the load test AND the jam test at
   * once -- the transient that once made the single-piece detector count shots nobody took. The
   * settle window is the only thing between it and a spurious unjam pulse on every burst, so this
   * case watches every loop of the start, not the end state.
   */
  @Test
  void aSpinUpFromRestIsNeitherLoadedNorJammed() {
    indexer.setWantedState(Indexer.WantedState.FEED);
    for (int i = 0; i < loadDetectionLoops(); i++) {
      step(1);
      assertFalse(indexer.isLoaded(), "spin-up read as a load at loop " + i);
      assertEquals(
          Indexer.SystemState.FEEDING,
          indexer.getSystemState(),
          "spin-up read as a jam at loop " + i);
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
  void deadHardwareTransitionsButDetectsNoPiece() {
    CommandScheduler.getInstance().unregisterSubsystem(indexer);
    indexer = new Indexer(new IndexerIO() {});

    indexer.setWantedState(Indexer.WantedState.INDEX);
    step(detectionLoops());

    assertEquals(
        Indexer.SystemState.INDEXING,
        indexer.getSystemState(),
        "the state machine must still run when the mechanism cannot");
    assertFalse(indexer.hasPiece(), "a dead indexer must not report a piece");
    assertEquals(0, indexer.getShotCount());
  }
}
