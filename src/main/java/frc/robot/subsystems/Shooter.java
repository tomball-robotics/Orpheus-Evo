// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final TalonFX topRoller = new TalonFX(Constants.SHOOTER.TOP_ROLLER_MOTOR_ID);
  private final TalonFX bottomRoller = new TalonFX(Constants.SHOOTER.BOTTOM_ROLLER_MOTOR_ID);

  private final VelocityVoltage topRollerVoltage = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage bottomRollerVoltage = new VelocityVoltage(0).withSlot(0);

  private final CoastOut topCoastOut = new CoastOut();
  private final CoastOut bottomCoastOut = new CoastOut();

  private double topRollerSetpoint = 0;
  private double bottomRollerSetpoint = 0;

  public Shooter() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = Constants.SHOOTER.VELOCITY_P;
    cfg.Slot0.kI = Constants.SHOOTER.VELOCITY_I;
    cfg.Slot0.kD = Constants.SHOOTER.VELOCITY_D;
    
    StatusCode topStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode bottomStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      topStatus = topRoller.getConfigurator().apply(cfg);
      bottomStatus = bottomRoller.getConfigurator().apply(cfg);
      if (topStatus.isOK() && bottomStatus.isOK()) break;
    }

    if (!(topStatus.isOK() && bottomStatus.isOK())) {
      System.out.println("Could not configure shooter motors. Top status: " + topStatus + " Bottom status: " + bottomStatus);
    }

    topRoller.setControl(topCoastOut);
    bottomRoller.setControl(bottomCoastOut);
  }

  private void setTopRollerVelocity(double velocity) {
    topRoller.setControl(topRollerVoltage.withVelocity(velocity));
    topRollerSetpoint = velocity;
    System.out.println("Top roller set to velocity: " + topRollerSetpoint + " RPS");
  }

  private void setBottomRollerVelocity(double velocity) {
    bottomRoller.setControl(bottomRollerVoltage.withVelocity(velocity));
    bottomRollerSetpoint = velocity;
    System.out.println("Bottom roller set to velocity: " + bottomRollerSetpoint + " RPS");
  }

  private void setBothRollersVelocity(double velocity) {
    setTopRollerVelocity(velocity);
    setBottomRollerVelocity(velocity);
  }

  public Command runShooterCommand(double velocity) {
    return new Command() {
      @Override
      public void initialize() {
        setBothRollersVelocity(velocity);
      }

      @Override
      public boolean isFinished() {
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        stopRollers();
        System.out.println("Shooter rollers stopped.");
      }
    };
  }

  /**
   * Sets the control mode for both shooter rollers to coast.
   */
  public void stopRollers() {
    topRollerSetpoint = 0;
    bottomRollerSetpoint = 0;
    topRoller.setControl(topCoastOut);
    bottomRoller.setControl(bottomCoastOut);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Top Roller RPS", topRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Bottom Roller RPS", bottomRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Top Roller Setpoint RPS", topRollerSetpoint);
    SmartDashboard.putNumber("Shooter/Bottom Roller Setpoint RPS", bottomRollerSetpoint);
  }
}
