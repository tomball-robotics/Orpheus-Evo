package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final SparkMax intakeLeft;
  private final SparkMax intakeRight;
  private final SparkClosedLoopController intakeRightPivotControl;
  private final RelativeEncoder pivotEncoder;

  private final TalonFX intakeRollers;
  private final DigitalInput limit;

  private final CoastOut coastOut = new CoastOut();
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  private double pivotSetpoint = 0;
  private double velocitySetpoint = 0;

  public Intake() {
    // Initialize motors
    intakeLeft = new SparkMax(Constants.INTAKE.LEFT_PIVOT_MOTOR_ID, MotorType.kBrushless);
    intakeRight = new SparkMax(Constants.INTAKE.RIGHT_PIVOT_MOTOR_ID, MotorType.kBrushless);
    intakeRightPivotControl = intakeRight.getClosedLoopController();
    pivotEncoder = intakeRight.getEncoder();

    // RIGHT pivot config with PID
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.INTAKE.PIVOT_P)
        .i(Constants.INTAKE.PIVOT_I)
        .d(Constants.INTAKE.PIVOT_D)
        .outputRange(-1, 1);

    intakeRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // LEFT follows RIGHT (invert if needed based on physical mounting)
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.follow(intakeRight, /* isInverted = */ true); // set to false if they spin the same way
    intakeLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Intake rollers (Falcon)
    intakeRollers = new TalonFX(Constants.INTAKE.ROLLER_MOTOR_ID);
    limit = new DigitalInput(Constants.INTAKE.INTAKE_LIMIT_SWITCH);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.Slot0.kP = Constants.INTAKE.ROLLER_VELOCITY_P;
    rollerConfig.Slot0.kI = Constants.INTAKE.ROLLER_VELOCITY_I;
    rollerConfig.Slot0.kD = Constants.INTAKE.ROLLER_VELOCITY_D;

    StatusCode status;
    for (int i = 0; i < 5; ++i) {
      status = intakeRollers.getConfigurator().apply(rollerConfig);
      if (status.isOK()) break;
    }

    intakeRollers.setControl(coastOut);

    // Dashboard controls
    SmartDashboard.setDefaultNumber("Intake/Pivot/Setpoint", 0);
    SmartDashboard.setDefaultBoolean("Intake/Pivot/Reset Encoder", false);
  }

  public Command setPivotPosition(double setpoint) {
    return new InstantCommand(() -> {
      pivotSetpoint = setpoint;
      intakeRightPivotControl.setReference(setpoint, ControlType.kPosition);
    }, this);
  }

  private void setRollerVelocity(double velocity) {
    intakeRollers.setControl(velocityVoltage.withVelocity(velocity));
    velocitySetpoint = velocity;
    System.out.println("Intake rollers set to velocity: " + velocitySetpoint + " RPS");
  }

  private void stopRollers() {
    intakeRollers.setControl(coastOut);
    velocitySetpoint = 0;
    System.out.println("Intake rollers stopped.");
  }

  public Command runRollers(double velocity) {
    return new Command() {
      @Override
      public void initialize() {
        setRollerVelocity(velocity);
      }

      @Override
      public boolean isFinished() {
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        stopRollers();
      }
    };
  }

  @Override
  public void periodic() {
    // Pivot telemetry
    SmartDashboard.putNumber("Intake/Pivot/Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Intake/Pivot/Current", intakeRight.getOutputCurrent());

    // Encoder reset toggle
    if (SmartDashboard.getBoolean("Intake/Pivot/Reset Encoder", false)) {
      pivotEncoder.setPosition(0);
      SmartDashboard.putBoolean("Intake/Pivot/Reset Encoder", false);
    }

    // Roller telemetry
    SmartDashboard.putNumber("Intake/Rollers/Velocity", intakeRollers.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/Rollers/Setpoint", velocitySetpoint);
    SmartDashboard.putNumber("Intake/Rollers/Supply Current", intakeRollers.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putString("Intake/Rollers/Control Mode", intakeRollers.getControlMode().getName());
    SmartDashboard.putBoolean("Intake/Rollers/Limit Switch", limit.get());

    if (limit.get()) {
      LED.noteIndexed();
    } else if (velocitySetpoint != 0) {
      LED.intakeRunning();
    } else if (!LED.isInEnabledState()) {
      LED.setEnabled();
    }
  }
}
