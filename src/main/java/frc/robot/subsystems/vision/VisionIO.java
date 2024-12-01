// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    Pose3d estimatedPose;
    // doesn't account for latency
    double timestamp;
    double latency;
    double tagCount;
    boolean inField;
    String camera;
    // null represents all ids
    double[] validIds;
    double averageTagDistance;
    // percentage of image
    double averageTagArea;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setValidIds(double[] validIds) {}

  public default void setPose(Pose3d pose) {}
}
