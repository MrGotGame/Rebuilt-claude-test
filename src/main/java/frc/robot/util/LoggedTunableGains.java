package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
  /**
   * Gravity feedforward. Not final, unlike the other six, because {@link #withGravity(double)}
   * replaces it to set a non-zero compiled default -- {@code initDefault} is idempotent and so
   * cannot raise a default that the constructor already latched at zero.
   */
  private LoggedTunableNumber kG;

  /**
   * How Phoenix should interpret the kG term.
   *
   * <p>Defaults to {@link GravityTypeValue#Elevator_Static}, which is a constant offset and is the
   * correct no-op behaviour when kG is left at zero. {@link #withGravity(double)} switches it to
   * {@link GravityTypeValue#Arm_Cosine} for rotating mechanisms, where the gravity term must scale
   * with the cosine of the arm angle.
   */
  private GravityTypeValue gravityType = GravityTypeValue.Elevator_Static;

  /** Dashboard prefix, retained so {@link #withGravity(double)} can rebuild the kG entry. */
  private final String prefix;

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
    // IMPLEMENTATION (was TODO: "Create one LoggedTunableNumber per gain under "<prefix>/kP" etc.,
    // with kG defaulted to 0.")
    //
    // kG defaults to 0 so that a mechanism which does not fight gravity -- a flywheel, a roller, a
    // swerve azimuth -- gets a gravity term that is exactly inert, and its owner never has to think
    // about it. Only withGravity() opts in.
    this.prefix = prefix;
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
    // IMPLEMENTATION (was TODO: "initDefault kG and return this, so arm/pivot mechanisms can opt in
    // fluently.")
    //
    // kG is REPLACED rather than re-defaulted. LoggedTunableNumber.initDefault is deliberately
    // idempotent, so calling it on the kG the constructor already latched at 0.0 would silently do
    // nothing -- a method named withGravity(g) that quietly ignores g is exactly the kind of trap
    // this codebase tries not to set.
    //
    // The gravity TYPE is switched at the same time: a rotating mechanism needs Arm_Cosine, where
    // the term scales with the cosine of the arm angle, not the constant offset that
    // Elevator_Static applies.
    this.kG = new LoggedTunableNumber(prefix + "/kG", g);
    this.gravityType = GravityTypeValue.Arm_Cosine;
    return this;
  }

  /**
   * Snapshots the current gain values into a Phoenix configuration object.
   *
   * @return a fresh {@link Slot0Configs} carrying the current values of all seven gains
   */
  public Slot0Configs toSlot0Configs() {
    // IMPLEMENTATION (was TODO: "Build a Slot0Configs with withKP/withKI/withKD/withKS/withKV/
    // withKA/withKG from the current tunable values. Verify the kG and GravityType setters against
    // Phoenix6 26.1.2 docs.")
    //
    // A FRESH Slot0Configs is returned every call rather than a cached mutable one. Phoenix's
    // configurator reads the object asynchronously, so handing out a shared instance that a later
    // call mutates is a genuine race.
    return new Slot0Configs()
        .withKP(kP.get())
        .withKI(kI.get())
        .withKD(kD.get())
        .withKS(kS.get())
        .withKV(kV.get())
        .withKA(kA.get())
        .withKG(kG.get())
        .withGravityType(gravityType);
  }

  /**
   * Applies a freshly-built configuration to hardware, but only when a gain actually changed.
   *
   * @param id caller identity; pass {@code hashCode()} of the owning subsystem
   * @param action receives the new configuration; should push it to the Talon
   *     <p><b>Outputs:</b> invokes {@code action} at most once per call.
   */
  public void ifChanged(int id, Consumer<Slot0Configs> action) {
    // IMPLEMENTATION (was TODO: "LoggedTunableNumber.ifChanged(id, () -> action.accept(
    // toSlot0Configs()), kP, kI, kD, kS, kV, kA, kG). The action should call tryUntilOk(...)")
    //
    // toSlot0Configs() is evaluated lazily INSIDE the runnable, so the (comparatively expensive)
    // config object is only built on the loops where something actually moved -- which, after the
    // first loop, is almost never.
    LoggedTunableNumber.ifChanged(
        id, () -> action.accept(toSlot0Configs()), kP, kI, kD, kS, kV, kA, kG);
  }

  /**
   * Reads the velocity feedforward gain directly, for feedforward computed in Java rather than on
   * the Talon.
   *
   * @return current kV, in volts per unit of velocity
   */
  public double kv() {
    // IMPLEMENTATION (was TODO: "return kV.get(); needed for open-loop velocity feedforward outside
    // Phoenix.")
    return kV.get();
  }

  /**
   * Reads the static friction gain directly, for feedforward computed in Java rather than on the
   * Talon.
   *
   * @return current kS, in volts
   */
  public double ks() {
    // IMPLEMENTATION (was TODO: "return kS.get().")
    return kS.get();
  }
}
