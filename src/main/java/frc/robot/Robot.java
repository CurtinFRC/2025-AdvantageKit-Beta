// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.BuildConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.index.IndexIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionSource;
import frc.robot.util.AutoChooser;
import java.io.File;
import java.util.Set;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private final double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final CommandXboxController driver = new CommandXboxController(0);

  private final Drive drive;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Index m_index;
  private final Superstructure superstructure;
  private final VisionSource limelight_back;
  private final VisionSource limelight_front;

  private final SwerveRequest.FieldCentric driveReq =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use closed-loop control for drive motors

  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;

  public Robot() {
    var LOG_DIRECTORY = new File("/U/logs");
    if (!LOG_DIRECTORY.exists()) {
      LOG_DIRECTORY.mkdirs();
    }

    SignalLogger.setPath(LOG_DIRECTORY.toString());
    SignalLogger.start();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter(LOG_DIRECTORY.toString()));
        Logger.addDataReceiver(new NT4Publisher());

        drive =
            new Drive(
                new DriveIOCTRE(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));

        m_shooter = new Shooter(new ShooterIOSparkMax());
        m_index = new Index(new IndexIOSparkMax());
        m_intake = new Intake(new IntakeIOSparkMax());

        limelight_back = new VisionSource(new VisionIOLimelight("limelight-back"));
        limelight_front = new VisionSource(new VisionIOLimelight("limelight-front"));
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());

        drive =
            new Drive(
                new DriveIOCTRE(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));

        m_shooter = new Shooter(new ShooterIOSim());
        m_index = new Index(new IndexIO() {});
        m_intake = new Intake(new IntakeIO() {});

        limelight_back = new VisionSource(new VisionIOSim("limelight-back"));
        limelight_front = new VisionSource(new VisionIOSim("limelight-front"));
        break;

        // in replay
      default:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

        // inputs come from log file
        drive = new Drive(new DriveIO() {});
        m_shooter = new Shooter(new ShooterIO() {});
        m_index = new Index(new IndexIO() {});
        m_intake = new Intake(new IntakeIO() {});
        limelight_back = new VisionSource(new VisionIO() {});
        limelight_front = new VisionSource(new VisionIO() {});
        break;
    }

    // Start AdvantageKit logger
    Logger.registerURCL(URCL.startExternal());
    Logger.start();

    superstructure = new Superstructure(m_shooter, m_intake, m_index);

    // TODO remove me and use LL LED abstraction
    NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ledMode").setNumber(1);

    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight-back.local", port);
      PortForwarder.add(port + 10, "limelight-front.local", port);
    }

    limelight_back.setCameraOffset(
        new Transform3d(
            Millimeters.of(320).unaryMinus(),
            Millimeters.of(200).unaryMinus(),
            Millimeters.of(0),
            Rotation3d.kZero)); // we only care about 2d translation data

    m_intake.setDefaultCommand(superstructure.intake());
    m_shooter.setDefaultCommand(m_shooter.stop());
    m_index.setDefaultCommand(m_index.stop());

    drive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drive.applyRequest(
            () ->
                driveReq
                    .withVelocityX(-driver.getLeftY() * MaxSpeed) // controller x is wpilib y
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // controller y is wpilib x
                    .withRotationalRate(
                        driver.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));
    // driver
    //     .leftBumper()
    //     .whileTrue(
    //         drive.driverAssistance(
    //             new Pose2d(2, 5.6, new Rotation2d()),
    //             () -> -driver.getLeftY() * MaxSpeed,
    //             () -> -driver.getLeftX() * MaxSpeed));
    driver.rightBumper().whileTrue(superstructure.shoot());
    driver.a().whileTrue(drive.brake());
    driver.y().onTrue(drive.seedFieldCentric());
    // driver
    //     .x()
    //     .whileTrue(
    //         drive
    //             .moveToPosition(new Pose2d(2, 5.6, new Rotation2d()))
    //             .until(drive::isTeleopAtSetpoint));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.back().and(driver.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    autoFactory =
        Choreo.createAutoFactory(
            drive,
            () -> drive.getState().Pose,
            drive::followPath,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            new AutoFactory.AutoBindings());

    autoChooser = new AutoChooser(autoFactory, "");

    autoChooser.addOption(
        "Simple Path",
        (factory) -> {
          final AutoLoop routine = factory.newLoop("simple");
          final AutoTrajectory path = factory.trajectory("SimplePath", routine);

          routine
              .enabled()
              .onTrue(
                  drive
                      .runOnce(
                          () ->
                              path.getInitialPose()
                                  .ifPresentOrElse(pose -> drive.resetPose(pose), routine::kill))
                      .andThen(path.cmd()));

          return routine.cmd();
        });

    autoChooser.addOption(
        "Bad Idea TM",
        (factory) -> {
          final AutoLoop routine = factory.newLoop("jadesadumbass");
          final AutoTrajectory path = factory.trajectory("VeryBadIdeaTm", routine);

          routine
              .enabled()
              .onTrue(
                  drive
                      .runOnce(
                          () ->
                              path.getInitialPose()
                                  .ifPresentOrElse(pose -> drive.resetPose(pose), routine::kill))
                      .andThen(path.cmd()));

          return routine.cmd();
        });

    autoChooser.addOption(
        "Move To Side",
        (factory) -> {
          final AutoLoop routine = factory.newLoop("side");
          final AutoTrajectory path = factory.trajectory("MoveToSide", routine);

          routine
              .enabled()
              .onTrue(
                  drive
                      .runOnce(
                          () ->
                              path.getInitialPose()
                                  .ifPresentOrElse(pose -> drive.resetPose(pose), routine::kill))
                      .andThen(path.cmd()));

          return routine.cmd();
        });

    autoChooser.addOption(
        "Test Path",
        (factory) -> {
          final AutoLoop routine = factory.newLoop("test");
          final AutoTrajectory path = factory.trajectory("TestPath", routine);

          routine
              .enabled()
              .onTrue(
                  drive
                      .runOnce(
                          () ->
                              path.getInitialPose()
                                  .ifPresentOrElse(pose -> drive.resetPose(pose), routine::kill))
                      .andThen(path.cmd()));

          return routine.cmd();
        });

    autoChooser.addOption(
        "Set Pose to Vision Pose",
        (factory) -> {
          return Commands.runOnce(
              () -> drive.resetPose(new Pose2d(1.3, 5.5, new Rotation2d())), drive);
        });

    autonomous().whileTrue(Commands.defer(() -> autoChooser.getSelected().asProxy(), Set.of()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    autoChooser.periodic();

    if (limelight_back.inField()) {
      drive.addVisionMeasurement(
          limelight_back.getPose().toPose2d(),
          limelight_back.getTimestamp(),
          limelight_back.getStdDevs());
    }

    if (limelight_front.inField()) {
      drive.addVisionMeasurement(
          limelight_front.getPose().toPose2d(),
          limelight_front.getTimestamp(),
          limelight_front.getStdDevs());
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    limelight_back.setPose(new Pose3d(drive.getState().Pose));
  }
}
