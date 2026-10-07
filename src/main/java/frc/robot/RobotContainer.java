// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class RobotContainer {
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController opController     = new CommandXboxController(1);

    /* Create subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Indexer  indexer  = new Indexer();
    public final Intake   intake   = new Intake();
    public final Launcher launcher = new Launcher();

    /* Custom triggers */
    public final Trigger launcherReady = new Trigger(() -> this.launcher.atTargetSpeed());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        /* default swereve stuff */
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        
        drivetrain.registerTelemetry(logger::telemeterize);
        
        /* Driver controls */
        driverController.a()
            .whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.y()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));// Reset the field-centric heading on left bumper press.
        driverController.rightBumper()
            .whileTrue(indexer.Feed());
        driverController.leftBumper()
            .onTrue(intake.Deploy().andThen(intake.Feed()));
        driverController.rightTrigger().and(launcherReady)
            .whileTrue(launcher.enableLauncher());

        /* opreator controls */
        opController.rightBumper()
            .whileTrue(indexer.BackFeed());
        opController.leftBumper()
            .onTrue(intake.Deploy().andThen(intake.UnFeed()))
            .onFalse(intake.Retract());
        opController.leftTrigger()
            .onTrue(intake.Deploy())
            .onFalse(intake.Retract());
        opController.povUp()
            .onTrue(launcher.bumpSpeed(10));
        opController.povDown()
            .onTrue(launcher.bumpSpeed(-10));
        opController.povLeft()
            .onTrue(launcher.bumpSpeed(-1));
        opController.povRight()
            .onTrue(launcher.bumpSpeed(1));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
