// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Everything lives in this one class on top of the existing swerve drivetrain:
 * intake, indexer, flywheel (shooter), turret, and hood.
 *
 * Two Xbox controllers: a DRIVER (port 0, sticks reserved for swerve) and an
 * OPERATOR (port 1) who manually drives the turret and hood live off their sticks.
 * Turret and hood are plain open-loop percent-output control - no PID, no setpoints.
 * Whatever the stick says, the motor does, right now. Let go of the stick and it stops
 * (idle mode brake holds position against gravity/friction, it just isn't "held" by PID).
 *
 * DRIVER CONTROLS (port 0):
 * A ................ Toggle shooter flywheel on/off (spins to target RPM, or spins down)
 * D-Pad Up .......... Increase flywheel target RPM
 * D-Pad Down ........ Decrease flywheel target RPM
 * Left Bumper (hold). Run intake + indexer forward (intake a note)
 * Right Bumper (hold) Run intake + indexer in reverse (eject / unjam)
 * Right Trigger ..... Feed indexer into flywheel (shoot), analog threshold
 *
 * OPERATOR CONTROLS (port 1):
 * Left Stick X ...... Turret left/right, proportional to stick deflection
 * Right Stick Y ..... Hood up/down, proportional to stick deflection
 */
public class Robot extends TimedRobot {

  // ==================================================================================
  // CONSTANTS - CAN IDs, tuning values, and button-mapped increments all live here
  // ==================================================================================
  private static final class Constants {
    // ---- CAN IDs (from Zach's message) ----
    static final int kIntakeCanId = 10;
    static final int kTurretCanId = 11;
    static final int kIndexerCanId = 12;
    static final int kFlywheelLeaderCanId = 13;
    static final int kFlywheelFollowerCanId = 14;
    static final int kHoodCanId = 15;

    // ---- Simple % power devices ----
    static final double kIntakePower = 1.0;
    static final double kIndexerFeedPower = 1.0;
    static final double kEjectPower = -1.0;

    // ---- Flywheel (velocity closed loop, RPM) ----
    static final double kFlywheelDefaultTargetRpm = 3000.0;
    static final double kFlywheelRpmIncrement = 250.0;
    static final double kFlywheelMinRpm = 0.0;
    static final double kFlywheelMaxRpm = 6000.0; // NEO 2.0 free speed ~5820 RPM at the motor, leave headroom

    // NOTE: these are placeholder gains. Tune kP first with SmartDashboard, then add kFF.
    // kFF should be ~= 1 / kV (in RPM units), i.e. output-per-RPM needed to hold speed.
    static final double kFlywheelP = 0.0002;
    static final double kFlywheelI = 0.0;
    static final double kFlywheelD = 0.0;
    static final double kFlywheelFF = 0.000175; // starting point, tune against real data

    static final double kFlywheelRpmTolerance = 100.0; // "at speed" band for driver feedback / interlocks

    // ---- Turret (manual open-loop, driven directly by operator's left stick X) ----
    static final double kTurretManualSpeed = 0.5; // max % power at full stick deflection
    static final double kTurretDeadband = 0.1;

    // ---- Hood (manual open-loop, driven directly by operator's right stick Y) ----
    static final double kHoodManualSpeed = 0.4; // max % power at full stick deflection
    static final double kHoodDeadband = 0.1;

    static final int kDriverControllerPort = 0;
    static final int kOperatorControllerPort = 1;
  }

  // ==================================================================================
  // HARDWARE
  // ==================================================================================
  private final XboxController driver = new XboxController(Constants.kDriverControllerPort);
  private final XboxController operator = new XboxController(Constants.kOperatorControllerPort);

  private final SparkMax intakeMotor = new SparkMax(Constants.kIntakeCanId, MotorType.kBrushless);
  private final SparkMax indexerMotor = new SparkMax(Constants.kIndexerCanId, MotorType.kBrushless);

  private final SparkMax flywheelLeader = new SparkMax(Constants.kFlywheelLeaderCanId, MotorType.kBrushless);
  private final SparkMax flywheelFollower = new SparkMax(Constants.kFlywheelFollowerCanId, MotorType.kBrushless);
  private final RelativeEncoder flywheelEncoder = flywheelLeader.getEncoder();
  private final SparkClosedLoopController flywheelController = flywheelLeader.getClosedLoopController();

  // Turret and hood are plain percent-output motors now - no closed loop controller needed.
  private final SparkMax turretMotor = new SparkMax(Constants.kTurretCanId, MotorType.kBrushless);
  private final SparkMax hoodMotor = new SparkMax(Constants.kHoodCanId, MotorType.kBrushless);

  // ==================================================================================
  // STATE
  // ==================================================================================
  private boolean shooterEnabled = false;
  private double flywheelTargetRpm = Constants.kFlywheelDefaultTargetRpm;

  // Used to detect rising edges (button just pressed this loop) since we aren't using
  // command-based Trigger bindings here.
  private boolean lastAButton = false;
  private int lastPov = -1;

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  // ==================================================================================
  // ROBOT LIFECYCLE
  // ==================================================================================

  @Override
  public void robotInit() {
    // TODO: keep your existing RobotContainer() call if that's where the swerve
    // drivetrain / REV swerve library setup lives. This file only adds the
    // scoring mechanisms on top of it.
    m_robotContainer = new RobotContainer();

    configureIntake();
    configureIndexer();
    configureFlywheel();
    configureTurret();
    configureHood();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Reset state each time teleop starts so we don't inherit a stale target from auto.
    shooterEnabled = false;
    flywheelTargetRpm = Constants.kFlywheelDefaultTargetRpm;
  }

  @Override
  public void teleopPeriodic() {
    // NOTE: swerve drive input handling is assumed to already be happening
    // elsewhere (e.g. inside your swerve subsystem's default command via
    // the command scheduler above). This method only handles the mechanisms
    // below, layered on top.

    handleShooterToggle();
    handleFlywheelRpmAdjust();
    handleManualTurretAndHood();
    handleIntakeAndIndexer();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  // ==================================================================================
  // CONFIGURATION - runs once in robotInit()
  // ==================================================================================

  private void configureIntake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false);
    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureIndexer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false);
    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureFlywheel() {
    // Leader: velocity closed loop on the motor's own encoder.
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kCoast).inverted(false); // coast so it doesn't brake-jerk to a stop
    leaderConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.kFlywheelP)
        .i(Constants.kFlywheelI)
        .d(Constants.kFlywheelD)
        .velocityFF(Constants.kFlywheelFF)
        .outputRange(-1.0, 1.0);
    flywheelLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Follower: no inversion needed per Zach, just mirrors the leader's output.
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.idleMode(IdleMode.kCoast);
    followerConfig.follow(Constants.kFlywheelLeaderCanId, false);
    flywheelFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureTurret() {
    // Plain percent-output, brake so it doesn't drift once the stick is centered.
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false);
    turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // TODO: once you know the mechanical range of motion, consider adding soft limits
    // off the built-in relative encoder so a stuck stick / runaway can't wind it up:
    // config.softLimit.forwardSoftLimit(...).forwardSoftLimitEnabled(true)
    //                  .reverseSoftLimit(...).reverseSoftLimitEnabled(true);
  }

  private void configureHood() {
    // Plain percent-output, brake so it doesn't sag once the stick is centered.
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).inverted(false);
    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // TODO: same as turret - add soft limits once the mechanical range is known.
  }

  // ==================================================================================
  // TELEOP BUTTON HANDLING
  // ==================================================================================

  /** A button: toggle the flywheel between spun-up-to-target and idle. */
  private void handleShooterToggle() {
    boolean aButton = driver.getAButton();
    if (aButton && !lastAButton) {
      shooterEnabled = !shooterEnabled;
    }
    lastAButton = aButton;

    if (shooterEnabled) {
      flywheelController.setReference(flywheelTargetRpm, ControlType.kVelocity);
    } else {
      flywheelController.setReference(0.0, ControlType.kVelocity);
    }
  }

  /** D-Pad Up/Down: raise or lower the flywheel's target RPM. */
  private void handleFlywheelRpmAdjust() {
    int pov = driver.getPOV();
    boolean povJustPressed = pov != -1 && pov != lastPov;

    if (povJustPressed) {
      if (pov == 0) { // Up
        flywheelTargetRpm += Constants.kFlywheelRpmIncrement;
      } else if (pov == 180) { // Down
        flywheelTargetRpm -= Constants.kFlywheelRpmIncrement;
      }
      flywheelTargetRpm = clamp(flywheelTargetRpm, Constants.kFlywheelMinRpm, Constants.kFlywheelMaxRpm);
    }
    lastPov = pov;
  }

  /**
   * Operator's left stick X drives the turret, right stick Y drives the hood.
   * Pure open-loop: whatever the stick reads (past deadband), that's the motor's
   * percent output, scaled down so it doesn't slew wildly. No PID, no setpoint.
   */
  private void handleManualTurretAndHood() {
    double turretInput = MathUtil.applyDeadband(operator.getLeftX(), Constants.kTurretDeadband);
    turretMotor.set(turretInput * Constants.kTurretManualSpeed);

    // Right stick Y is typically positive when pushed down, so invert it to make
    // "stick up" mean "hood up".
    double hoodInput = MathUtil.applyDeadband(-operator.getRightY(), Constants.kHoodDeadband);
    hoodMotor.set(hoodInput * Constants.kHoodManualSpeed);
  }

  /**
   * Left Bumper: intake in. Right Bumper: eject/reverse.
   * Right Trigger: feed the indexer into the flywheel to shoot.
   * Right Trigger wins if held at the same time as a bumper.
   */
  private void handleIntakeAndIndexer() {
    boolean intakeIn = driver.getLeftBumperButton();
    boolean intakeReverse = driver.getRightBumperButton();
    boolean feedToShoot = driver.getRightTriggerAxis() > 0.5;

    if (feedToShoot) {
      indexerMotor.set(Constants.kIndexerFeedPower);
      intakeMotor.set(Constants.kIntakePower);
    } else if (intakeIn) {
      intakeMotor.set(Constants.kIntakePower);
      indexerMotor.set(Constants.kIndexerFeedPower);
    } else if (intakeReverse) {
      intakeMotor.set(Constants.kEjectPower);
      indexerMotor.set(Constants.kEjectPower);
    } else {
      intakeMotor.set(0.0);
      indexerMotor.set(0.0);
    }
  }

  /** True once the flywheel is within tolerance of its target RPM - handy for driver feedback. */
  private boolean isFlywheelAtSpeed() {
    return shooterEnabled
        && Math.abs(flywheelEncoder.getVelocity() - flywheelTargetRpm) <= Constants.kFlywheelRpmTolerance;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}