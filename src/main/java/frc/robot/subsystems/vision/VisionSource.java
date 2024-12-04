// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class VisionSource extends SubsystemBase
/** this isnt a subsystem but we need the periodic */
{
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public VisionSource(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("VisionSource/" + inputs.camera, inputs);
  }

  /** Pose with camera offset accounted for */
  public Pose3d getPose() {
    if (inputs.cameraOffset != null) {
      return inputs.estimatedPose.plus(inputs.cameraOffset);
    } else {
      return inputs.estimatedPose;
    }
  }

  public boolean inField() {
    return inputs.inField;
  }

  /** Latency compensated timestamp */
  public double getTimestamp() {
    return inputs.timestamp - inputs.latency;
  }

  // TODO account for gyro rotational rate, tag area, tag distance
  public Matrix<N3, N1> getStdDevs() {
    if (inputs.tagCount <= 0) {
      return VecBuilder.fill(9999999, 9999999, 9999999);
    }
    return VecBuilder.fill(0.7, 0.7, 9999999);
  }

  public void setCameraOffset(Transform3d translation) {
    io.setCameraOffset(translation);
  }

  /** for sim */
  public void setPose(Pose3d pose) {
    if (!RobotBase.isSimulation()) {
      throw new RuntimeException("Can't set vision pose out of sim");
    }
    io.setPose(pose);
    inputs.estimatedPose = pose;
  }
}
