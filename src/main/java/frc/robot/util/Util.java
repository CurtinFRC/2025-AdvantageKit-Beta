// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.util;

import static edu.wpi.first.units.Units.Microseconds;
import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.WPIUtilJNI;

public final class Util {
  private Util() {}

  public static Time now() {
    return Microseconds.of(WPIUtilJNI.now());
  }

  public static boolean inField(Pose3d pose) {
    return ((pose.getX() < FIELD_X)
        && (pose.getX() > 0)
        && (pose.getY() < FIELD_Y)
        && (pose.getY() > 0));
  }
}
