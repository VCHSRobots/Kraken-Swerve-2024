// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.SwerveFieldCentricFacingAngleProfiledRequest;
import frc.robot.Util.Util;
import frc.robot.generated.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // drivetrain

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Double> speedChooser = new SendableChooser<>();
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Initial max is true top speed
  private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
  private final double MaxAngularRate = Math.PI * 4.0; // .75 rotation per second max angular velocity. Adjust for max
                                                       // turning rate speed.
  private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity. Adjust for
                                                          // max turning rate speed.
  private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(1); // operator xbox controller
  private final CommandXboxController m_testController = new CommandXboxController(2);
  private final CommandXboxController m_sysidController = new CommandXboxController(3);

  // Slew Rate Limiters to limit acceleration of joystick inputs
  // not used by default for more driver control.
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(0.3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(0.3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.3);

  // Field-centric driving in Open Loop, can change to closed loop after
  // characterization
  // withDeadbands force requested speeds lower than that value to 0.
  // TODO since the deadband in on the joystick input values, we should play with
  // setting
  // no deadband (0)
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(MaxSpeed * 0.005)
      .withRotationalDeadband(AngularRate * 0.005);
  SwerveFieldCentricFacingAngleProfiledRequest driveLockedAngle = new SwerveFieldCentricFacingAngleProfiledRequest()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(MaxSpeed * 0.005)
      .withRotationalDeadband(AngularRate * 0.005)
      .withHeadingController(new ProfiledPIDController(5, 0, 0, new Constraints(Math.PI * 2.0, Math.PI * 4.0)));

  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  Telemetry logger = new Telemetry(MaxSpeed);

  Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

  private Supplier<SwerveRequest> controlStyle = () -> {
    return drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward -Y
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-m_driverController.getRightX() * AngularRate); // Drive counterclockwise with negative X
                                                                            // (left);
  };
  private Double lastSpeed = 0.65;
  private boolean isSwerveLockingAngle = false;
  private Rotation2d lockedAngle = new Rotation2d();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // left y shooter speed
    // m_shooter.setDefaultCommand(new RunCommand(() -> {
    // double leftY = m_driverController.getLeftY();
    // if (Math.abs(leftY) > 0.04) {
    // m_shooter.shoot(m_driverController.getLeftY());
    // } else {
    // m_shooter.shoot(0);
    // }
    // }, m_shooter));

    // m_pivot.setDefaultCommand(m_pivot.getHoldPositionCommand());

    // right bumper lower pivot

    // right bumper raise pivot


    // right trigger intake speed

    // left trigger reverse speed

    // m_driverController.y().whileTrue(m_pivot.getGotoPositionCommand(POSITION.SOURCE));

    m_driverController.rightStick()
        .toggleOnTrue(
            Commands.startEnd(() -> {
              // lockedAngle = drivetrain.getState().Pose.getRotation();
              lockedAngle = new Rotation2d();
              isSwerveLockingAngle = true;
            }, () -> isSwerveLockingAngle = false));

    // increase shooter speed
    // increase shooter speed
    // on/off shooter


    // zero

    // default commands
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          SmartDashboard.putNumber("locked angle", lockedAngle.getDegrees());
          double cubicWeight = 0.75;
          var x = Util.cubic(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1), cubicWeight);
          var y = Util.cubic(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1), cubicWeight);
          var r = Util.cubic(MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1), cubicWeight);

          // stop locking angle if at target
          if (Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - lockedAngle.getDegrees()) < 3) {
            isSwerveLockingAngle = false;
          }
          if (!isSwerveLockingAngle) {
            return drive.withVelocityX(x * MaxSpeed) // Drive forward -Y
                .withVelocityY(y * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(r * AngularRate); // Drive counterclockwise with negative X (left);
          } else {
            return driveLockedAngle.withVelocityX(x * MaxSpeed) // Drive forward -Y
                .withVelocityY(y * MaxSpeed) // Drive left with negative X (left)
                .withTargetDirection(lockedAngle); // Drive counterclockwise with negative X (left);
          }
        }).ignoringDisable(true));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // driver controller
    // TODO "reset odometry" should set rotation to 0 on blue side, but 180 deg on
    // red
    // TODO default reset should not set x,y unless reseting at a known location
    // (driver has robot at specified location)
    m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));
    m_driverController.pov(270).whileTrue(drivetrain.applyRequest(() -> {
      double x = 0;
      double y = 0;
      double r = 0;
      double maxSpeed = 2.0;
      Pose2d goal = new Pose2d(new Translation2d(1.5, 0), new Rotation2d()); // TODO check whether 180

      return forwardStraight.withDeadband(.05).withVelocityX(x).withVelocityY(y)
          .withRotationalRate(Units.degreesToRadians(r));
    }));

    // testing controller
    m_testController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_testController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-m_testController.getLeftY(), -m_testController.getLeftX()))));
    m_testController.x().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d())));
    m_testController.y()
        .whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(90))));

    m_sysidController.x().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    m_sysidController.x().and(m_sysidController.pov(180)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    m_sysidController.y().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    m_sysidController.y().and(m_sysidController.pov(180)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    m_sysidController.a().and(m_sysidController.pov(0)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    m_sysidController.a().and(m_sysidController.pov(180)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    m_sysidController.b().and(m_sysidController.pov(0)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    m_sysidController.b().and(m_sysidController.pov(180)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));

    // Drivetrain needs to be placed against a sturdy wall and test stopped
    // immediately upon wheel slip
    m_sysidController.back().and(m_sysidController.pov(0)).whileTrue(drivetrain.runDriveSlipTest());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();

  }

  private void newSpeed() {
    lastSpeed = speedChooser.getSelected();
    MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * lastSpeed;
  }}
