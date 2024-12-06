package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {
  @AutoLog
  public static class IndexIOInputs {
    public double position;
    public double velocity;
    public double appliedVolts;
    public double currentAmps;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}
}
