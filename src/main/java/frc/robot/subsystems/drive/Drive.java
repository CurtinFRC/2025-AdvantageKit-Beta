// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs;
  private final SwerveDriveState state;

  private final PIDController m_pathXController = new PIDController(10, 0, 0);
  private final PIDController m_pathYController = new PIDController(10, 0, 0);
  private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

  private final PIDController m_teleopXController = new PIDController(10, 0, 0.1);
  private final PIDController m_teleopYController = new PIDController(10, 0, 0.1);
  private final PIDController m_teleopThetaController = new PIDController(7, 0, 0.1);

  public boolean isTeleopAtSetpoint() {
    return m_teleopXController.atSetpoint()
        && m_teleopYController.atSetpoint()
        && m_teleopThetaController.atSetpoint();
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              Seconds.of(6),
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private final SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  private boolean hasAppliedOperatorPerspective;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BLUE_PERSPECTIVE = Rotation2d.fromDegrees(0);

  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RED_PERSPECTIVE = Rotation2d.fromDegrees(180);

  public Drive(DriveIO io) {
    this.io = io;
    state = new SwerveDriveState();
    inputs = new DriveIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    state.FailedDaqs = inputs.failedDaqs;
    state.ModuleStates = inputs.moduleStates;
    state.ModuleTargets = inputs.moduleTargetStates;
    state.OdometryPeriod = inputs.odometryPeriodSeconds;
    state.Pose = inputs.pose;
    state.SuccessfulDaqs = inputs.successfulDaqs;
    state.Speeds = inputs.speeds;

    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                io.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? RED_PERSPECTIVE : BLUE_PERSPECTIVE);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> io.setControl(requestSupplier.get()));
  }

  public Command brake() {
    return applyRequest(() -> brakeRequest);
  }

  public SwerveDriveState getState() {
    return state;
  }

  /**
   * Follows the given field-centric path sample with PID.
   *
   * @param pose Current pose of the robot
   * @param reference Sample along the path to follow
   */
  public void followPath(Pose2d pose, SwerveSample reference) {
    Logger.recordOutput("Drive/Reference Sample", reference);

    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    var targetSpeeds = reference.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), reference.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), reference.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), reference.heading);

    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(reference.moduleForcesX())
            .withWheelForceFeedforwardsY(reference.moduleForcesY()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public Command moveToPosition(Pose2d position) {
    var translationTolerance = 0.01;
    var rotationTolerance = 0.015;
    m_teleopXController.setTolerance(translationTolerance);
    m_teleopYController.setTolerance(translationTolerance);
    m_teleopThetaController.setTolerance(rotationTolerance);

    return run(
        () -> {
          Logger.recordOutput("Drive/Reference Pose", position);
          var pose = state.Pose;
          var x = m_teleopXController.calculate(pose.getX(), position.getX());
          var y = m_teleopYController.calculate(pose.getY(), position.getY());
          var theta =
              m_teleopThetaController.calculate(
                  pose.getRotation().getRadians(), position.getRotation().getRadians());
          setControl(
              new SwerveRequest.FieldCentric()
                  .withVelocityX(-x)
                  .withVelocityY(-y)
                  .withRotationalRate(theta));
        });
  }

  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  public Command seedFieldCentric() {
    return runOnce(() -> io.seedFieldCentric());
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    io.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    io.addVisionMeasurement(visionMeasurement, timestampSeconds, visionMeasurementStdDevs);
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }
}
