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
