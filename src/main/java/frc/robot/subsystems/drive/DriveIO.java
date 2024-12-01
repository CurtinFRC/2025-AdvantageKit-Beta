package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    public SwerveModuleState[] moduleTargetStates = new SwerveModuleState[4];
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public double odometryPeriodSeconds = 0.0;
    public int successfulDaqs = 0;
    public int failedDaqs = 0;

    public double gyroRate = 0.0;
    public Rotation3d rotation3d = new Rotation3d();
    public boolean odometryIsValid = false;
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  public default void setControl(SwerveRequest request) {}

  public default void seedFieldCentric() {}

  public default void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {}

  public default void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

  public default void resetPose(Pose2d pose) {}
}
