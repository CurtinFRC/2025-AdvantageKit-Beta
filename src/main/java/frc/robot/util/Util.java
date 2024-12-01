package frc.robot.util;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.WPIUtilJNI;

public class Util {
  private Util() {}

  /** Time in milliseconds */
  public static double now() {
    return WPIUtilJNI.now() / 1000;
  }

  public static boolean inField(Pose3d pose) {
    return ((pose.getX() < FIELD_X)
        && (pose.getX() > 0)
        && (pose.getY() < FIELD_Y)
        && (pose.getY() > 0));
  }
}
