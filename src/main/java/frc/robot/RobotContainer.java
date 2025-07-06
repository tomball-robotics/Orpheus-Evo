// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.annotation.processing.SupportedOptions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

@SuppressWarnings("unused")
public class RobotContainer {

    /* Swerve Config  */

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.CONTROL.STICK_DEADBAND).withRotationalDeadband(MaxAngularRate * Constants.CONTROL.STICK_DEADBAND) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Controls */

    private final CommandXboxController driverController = new CommandXboxController(Constants.CONTROL.DRIVER_CONTROLLER_PORT_ID);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.CONTROL.OPERATOR_CONTROLLER_PORT_ID);

    /* Subsystems */

    public final Swerve swerve = TunerConstants.createDrivetrain();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public final Climbers climbers = new Climbers();
    public final LED led = new LED();

    /* Commands */
    
    /* ETC */

    private final Telemetry logger = new Telemetry(MaxSpeed);


    public RobotContainer() {
        configureBindings();   
    }

    private void configureBindings() {

        // Climber up when you press bumper, and down when you press trigger.
        climbers.setDefaultCommand(
            climbers.setClimberSpeed(
                (driverController.leftBumper().getAsBoolean() ? Constants.CLIMBER.CLIMBER_SPEED : 0.0) - driverController.getLeftTriggerAxis()/2,
                (driverController.rightBumper().getAsBoolean() ? Constants.CLIMBER.CLIMBER_SPEED : 0.0) - driverController.getRightTriggerAxis()/2
            )
        );

        // Runs the shooter to the target velcocity while the right bumper is pressed, then returns to coast when released.
        operatorController.rightTrigger().whileTrue(
            shooter.runShooterCommand(Constants.SHOOTER.TARGET_VELOCITY) // Run shooter at target velocity when right bumper is pressed;
        );

        operatorController.leftBumper().onTrue(intake.setPivotPosition(Constants.INTAKE.PIVOT_INTAKE_ANGLE));
        operatorController.leftBumper().whileTrue(intake.setRollerVelocity(Constants.INTAKE.INTAKE_ROLLER_VELOCITY));
        operatorController.leftBumper().onFalse(intake.setPivotPosition(Constants.INTAKE.PIVOT_STOW_ANGLE));

        operatorController.rightBumper().onTrue(intake.setPivotPosition(Constants.INTAKE.PIVOT_AMP_ANGLE));
        operatorController.rightBumper().onFalse(intake.setPivotPosition(Constants.INTAKE.PIVOT_STOW_ANGLE));

        operatorController.leftTrigger().whileTrue(intake.setRollerVelocity(Constants.INTAKE.AMP_ROLLER_VELOCITY));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerve.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(swerve.applyRequest(() -> brake));
        // driverController.b().whileTrue(swerve.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));


        driverController.back().and(driverController.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.povUp().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        swerve.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
