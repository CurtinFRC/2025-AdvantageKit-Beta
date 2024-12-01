// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.vision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.Util;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable impl;
  private final DoubleArraySubscriber botpose;
  double[] validIds;
  private final String camera;

  public VisionIOLimelight(String camera) {
    this.camera = camera;
    impl = NetworkTableInstance.getDefault().getTable(camera);
    botpose = impl.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var pose = botpose.get();
    var rotation =
        new Rotation3d(
            degreesToRadians(pose[3]), degreesToRadians(pose[4]), degreesToRadians(pose[5]));
    inputs.estimatedPose = new Pose3d(pose[0], pose[1], pose[2], rotation);
    inputs.timestamp = Util.now();
    inputs.latency = pose[6];
    inputs.tagCount = pose[7];
    inputs.inField = Util.inField(inputs.estimatedPose);
    inputs.camera = camera;
    inputs.validIds = validIds;
    inputs.averageTagDistance = pose[9];
    inputs.averageTagArea = pose[10];
  }

  @Override
  public void setValidIds(double[] validIds) {
    impl.getDoubleArrayTopic("fiducial_id_filters_set").publish().set(validIds);
    this.validIds = validIds.clone();
  }
}
