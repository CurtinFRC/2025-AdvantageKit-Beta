package frc.robot;

public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static final double FIELD_X = 16.56588;
  public static final double FIELD_Y = 8.160512;
}
