package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import java.util.function.Consumer;

/**
 * A bundle of the seven closed-loop gains (kP, kI, kD, kS, kV, kA, kG) as one tunable group.
 *
 * <p>Every closed-loop mechanism in this project -- the intake pivot, the turret, the flywheel and
 * the four drive modules -- needs the same seven-tunable boilerplate plus a "reapply the config
 * only when something moved" guard. Bundling it here means a subsystem writes one line, {@code
 * gains.ifChanged(hashCode(), io::setGains)}, instead of seven. Getting that boilerplate wrong in
 * one place means a gain silently never reaches the Talon, which is very hard to see from the
 * driver station.
 */
public class LoggedTunableGains {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kG;

  /**
   * Creates the seven backing tunables under a shared dashboard prefix, with gravity feedforward
   * defaulted to zero.
   *
   * @param prefix dashboard prefix for this gain set, for example {@code "Turret"}; each gain
   *     appears as {@code /Tuning/<prefix>/kP} and so on
   * @param p proportional gain default
   * @param i integral gain default
   * @param d derivative gain default
   * @param s static friction feedforward default, in volts
   * @param v velocity feedforward default, in volts per unit of velocity
   * @param a acceleration feedforward default, in volts per unit of acceleration
   */
  public LoggedTunableGains(
      String prefix, double p, double i, double d, double s, double v, double a) {
    // TODO: Create one LoggedTunableNumber per gain, keyed "<prefix>/kP", "<prefix>/kI" and so on.
    // Default kG to 0.0 so mechanisms that do not fight gravity need not think about it.
    this.kP = new LoggedTunableNumber(prefix + "/kP", p);
    this.kI = new LoggedTunableNumber(prefix + "/kI", i);
    this.kD = new LoggedTunableNumber(prefix + "/kD", d);
    this.kS = new LoggedTunableNumber(prefix + "/kS", s);
    this.kV = new LoggedTunableNumber(prefix + "/kV", v);
    this.kA = new LoggedTunableNumber(prefix + "/kA", a);
    this.kG = new LoggedTunableNumber(prefix + "/kG", 0.0);
  }

  /**
   * Opts this gain set into gravity feedforward, for arm-like mechanisms such as the intake pivot.
   *
   * @param g gravity feedforward default, in volts
   * @return this instance, so the call can be chained onto the constructor
   */
  public LoggedTunableGains withGravity(double g) {
    // TODO: kG.initDefault(g) and return this. initDefault is a no-op if a default was already
    // latched, which is what makes this safe to call fluently.
    return this;
  }

  /**
   * Snapshots the current gain values into a Phoenix configuration object.
   *
   * @return a fresh {@link Slot0Configs} carrying the current values of all seven gains
   */
  public Slot0Configs toSlot0Configs() {
    // TODO: Build a Slot0Configs using withKP/withKI/withKD/withKS/withKV/withKA/withKG from the
    // current tunable values. Verify the kG setter and the GravityType enum against the Phoenix6
    // 26.1.2 docs -- an arm needs GravityType.Arm_Cosine, an elevator needs Elevator_Static.
    return new Slot0Configs();
  }

  /**
   * Applies a freshly-built configuration to hardware, but only when a gain actually changed.
   *
   * @param id caller identity; pass {@code hashCode()} of the owning subsystem
   * @param action receives the new configuration; should push it to the Talon
   *     <p><b>Outputs:</b> invokes {@code action} at most once per call.
   */
  public void ifChanged(int id, Consumer<Slot0Configs> action) {
    // TODO: LoggedTunableNumber.ifChanged(id, () -> action.accept(toSlot0Configs()), kP, kI, kD,
    // kS,
    // kV, kA, kG). The action passed in should call PhoenixUtil.tryUntilOk(5, () ->
    // talon.getConfigurator().apply(configs, 0.25)) so a dropped CAN frame does not silently lose
    // the new gains.
  }

  /**
   * Reads the velocity feedforward gain directly, for feedforward computed in Java rather than on
   * the Talon.
   *
   * @return current kV, in volts per unit of velocity
   */
  public double kv() {
    // TODO: return kV.get().
    return 0.0;
  }

  /**
   * Reads the static friction gain directly, for feedforward computed in Java rather than on the
   * Talon.
   *
   * @return current kS, in volts
   */
  public double ks() {
    // TODO: return kS.get().
    return 0.0;
  }
}
