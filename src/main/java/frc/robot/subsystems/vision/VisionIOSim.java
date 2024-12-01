// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

// TODO make me closer to reality
public class VisionIOSim implements VisionIO {
  private final String cameraName;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  public static final VisionSystemSim visionSim = new VisionSystemSim("main");

  static {
    AprilTagFieldLayout tagLayout;
    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException("Couldn't load AprilTagField layout");
    }
    visionSim.addAprilTags(tagLayout);
  }

  public VisionIOSim(String camera) {
    this.cameraName = camera;
    this.camera = new PhotonCamera(cameraName);

    // PV example properties
    SimCameraProperties cameraProp = new SimCameraProperties();
    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    // The simulation of this camera. Its values used in real robot code will be updated.
    cameraSim = new PhotonCameraSim(this.camera, cameraProp);
    visionSim.addCamera(cameraSim, new Transform3d());

    cameraSim.enableDrawWireframe(true); // disable on weak systems
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.camera = cameraName;

    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.getMultiTagResult().isPresent()) {
        inputs.estimatedPose =
            new Pose3d()
                .plus(
                    result
                        .getMultiTagResult()
                        .get()
                        .estimatedPose
                        .best); // TODO we should check this exists but its sim so idc
        inputs.timestamp =
            result.getTimestampSeconds() * 100; // TODO this does account for latency lol
        inputs.latency = 0; // hack to avoid issue above
        inputs.tagCount = result.getMultiTagResult().get().fiducialIDsUsed.size();
        // TODO add these
        inputs.averageTagDistance = 0;
        inputs.averageTagArea = 0;
      } else {
        inputs.tagCount = 0;
      }
    }
  }

  @Override
  public void setPose(Pose3d pose) {
    visionSim.update(pose);
  }
}
