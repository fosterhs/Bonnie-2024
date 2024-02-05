package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
  private final TalonFX leftClimbMotor = new TalonFX(9); // The motor that controls the left climber.
  private final TalonFX rightClimbMotor = new TalonFX(10); // The other that controls the right climber.
  private final double motorCurrentLimit = 40.0; // Motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 3; // The number of times a motor will attempt to reconfigure before declaring a failure and putting the device into a manual state.

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean leftClimbMotorFailure = false; 
  private boolean rightClimbMotorFailure = false; 

  public Climber() {
    leftClimbMotorFailure = !configMotor(leftClimbMotor, leftClimbMotorFailure);
    rightClimbMotorFailure = !configMotor(rightClimbMotor, rightClimbMotorFailure);
  }

  // Updates any important values on the dashboard.
  public void periodic() {
    SmartDashboard.putBoolean("Climber Failure", getMotorFailure());
  }

  // Sets the output of each climb motor. the ClimbPower inputs can range from -1 to 1. -1 corresponds to full power down and +1 is full power up.
  public void set(double leftClimbPower, double rightClimbPower) {
    if (!leftClimbMotorFailure) {
      leftClimbMotor.setControl(new DutyCycleOut(leftClimbPower));
    }
    if (!rightClimbMotorFailure) {
      rightClimbMotor.setControl(new DutyCycleOut(rightClimbPower));
    }
  }
  
  // Attempts to reboot the climber by reconfiguring the motors. Use if trying to troubleshoot a climber failure during a match.
  public void reboot() {
    leftClimbMotorFailure = !configMotor(leftClimbMotor, leftClimbMotorFailure);
    rightClimbMotorFailure = !configMotor(rightClimbMotor, rightClimbMotorFailure);
  }

  // Returns true if either of the motors failed to configure on startup or reboot.
  public boolean getMotorFailure() {
    return leftClimbMotorFailure || rightClimbMotorFailure;
  }

  // Sets PID constants, brake mode, inverts, and enforces a 40 A current limit. Returns true if the motor successfully configured.
  private boolean configMotor(TalonFX _motor, boolean motorFailure) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = _motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Inverts the direction of positive motor velocity.

    // Setting current limits
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = motorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentThreshold = motorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;

    // Velocity PIDV constants for reaching flywheel velocities
    motorConfigs.Slot0.kP = 0.008;
    motorConfigs.Slot0.kI = 0.06;
    motorConfigs.Slot0.kD = 0.0002;
    motorConfigs.Slot0.kV = 0.009;

    // Motion Magic Parameters for moving set distances
    motorConfigs.Slot1.kP = 0.8;
    motorConfigs.Slot1.kI = 2.0;
    motorConfigs.Slot1.kD = 0.006;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 75.0;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    motorConfigs.MotionMagic.MotionMagicJerk = 400.0;
    
    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int motorErrors = 0;
    while (motorConfigurator.apply(motorConfigs, 0.03) != StatusCode.OK) {
        motorErrors++;
      motorFailure = motorErrors > maxMotorFailures;
      if (motorFailure) {
        return false;
      }
    }
    return true;
  }   
}