// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  SparkMax intakeLeft;
  SparkMax intakeRight;
  SparkMaxConfig rightConfig;
  SparkMaxConfig leftConfig;
  SparkClosedLoopController intakeRightPivotControl;
  double pivotSetpoint = 0;

  TalonFX intakeRollers;
  TalonFXConfiguration rollerConfig;
  CoastOut coastOut = new CoastOut();
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  double velocitySetpoint = 0;
  
  public Intake() {
    intakeLeft = new SparkMax(Constants.INTAKE.LEFT_PIVOT_MOTOR_ID, MotorType.kBrushless);
    intakeRight = new SparkMax(Constants.INTAKE.RIGHT_PIVOT_MOTOR_ID, MotorType.kBrushless);
    
    leftConfig = new SparkMaxConfig();
    leftConfig.follow(intakeRight);
    intakeLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeRightPivotControl = intakeRight.getClosedLoopController();
    rightConfig = new SparkMaxConfig();
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(Constants.INTAKE.PIVOT_P)
    .i(Constants.INTAKE.PIVOT_I)
    .d(Constants.INTAKE.PIVOT_D)
    .outputRange(-1, 1);

    intakeRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rollerConfig = new TalonFXConfiguration();

    rollerConfig.Slot0.kP = Constants.INTAKE.ROLLER_VELOCITY_P;
    rollerConfig.Slot0.kI = Constants.INTAKE.ROLLER_VELOCITY_I;
    rollerConfig.Slot0.kD = Constants.INTAKE.ROLLER_VELOCITY_D;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeRollers.getConfigurator().apply(rollerConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

  }

  public Command setPivotPosition(double setpoint){
    return new Command() {
      @Override
      public void initialize() {
        intakeRightPivotControl.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        pivotSetpoint = setpoint;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

      @Override
      public void end(boolean interrupted) {}
    };
  }

  public Command setRollerVelocity(double velocity){
    return new Command() {
      @Override
      public void initialize() {
        intakeRollers.setControl(velocityVoltage.withVelocity(velocity));
        velocitySetpoint = velocity;
      }

      @Override
      public boolean isFinished() {
        return true;
      }

      @Override
      public void end(boolean interrupted) {
        intakeRollers.setControl(coastOut);
      }
    };
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot/Primary Encoder Position", intakeRight.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Intake/Pivot/Output Current", intakeRight.getOutputCurrent());

    SmartDashboard.putNumber("Intake/Rollers/Velocity", intakeRollers.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Setpoint", velocitySetpoint);
    SmartDashboard.putNumber("Intake/Rollers/Supply Current", intakeRollers.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Intake/Rollers/Control Mode", intakeRollers.getControlMode().getName());

  }
}
