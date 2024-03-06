package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
  private final TalonFX armMotorL = new TalonFX(12); // One of the motors that controls the arm.
  private final TalonFX armMotorR = new TalonFX(11); // One of the motors that controls the arm.
  private final double motorCurrentLimit = 40.0; // Motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 3; // The number of times a motor will attempt to reconfigure before declaring a failure and putting the device into a manual state.

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean armMotorLFailure = false; 
  private boolean armMotorRFailure = false;

  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0); // Keeps track of the angle of the arm.
  private double armEncoderZero = 0.518; // The initial arm position reading of the encoder in rotations.
  private double armSetpoint = 75.0; // The last requested setpoint of the arm in degrees. 0 degrees is horizontal and 90 degrees is vertical. 
  private final double armTol = 0.2; // The acceptable error in the angle of the arm in degrees.
  private final double gearRatio = 288.0; // 72:12 chain. 3:1, 4:1, and 4:1 stacked planetaries.
  private final double lowLimit = -3.0; // The lower limit of the arm in degrees.
  private final double highLimit = 75.0; // The higher limit of the arm in degrees.

  private boolean manualControl = false; // Indicates whether the arm is under manual control. This can happen if there is a motor failure, or if the operator requests it via setManualControl().
  private double manualPower = 0.0; // Stores the desired output of the arm motor when it is under manual control.

  private double armMotorInitialPos = 0.0; // The position of the arm motor on boot() in falcon rotations.
  private double armEncoderInitialPos = 0.0; // The position of the arm encoder on boot() in degrees, with a zero offset applied.
  private boolean atSetpoint = false;
  private boolean isCalibrated = false;

  public Arm() {
    reboot();
  }

  // Should be called once teleopPeriodic() and autoPeriodic() sections of the main robot code. Neccesary for the class to function.
  public void periodic() {
    if (manualControl) {
      atSetpoint = false;
      if (!getMotorFailure()) {
        armMotorL.setControl(new DutyCycleOut(manualPower).withEnableFOC(true));
        armMotorR.setControl(new Follower(12, true));
      } else if (!armMotorLFailure) {
        armMotorL.setControl(new DutyCycleOut(manualPower).withEnableFOC(true));
      } else if (!armMotorRFailure) {
        armMotorR.setControl(new DutyCycleOut(manualPower).withEnableFOC(true));
      }
    } else {
      manualPower = 0.0;
      double setpoint = armMotorInitialPos + (armSetpoint-armEncoderInitialPos)*gearRatio/360.0;
      if (!getMotorFailure()) {
        armMotorL.setControl(new MotionMagicDutyCycle(setpoint).withSlot(1).withEnableFOC(true));
        armMotorR.setControl(new Follower(12, true));
        atSetpoint = Math.abs(armMotorL.getRotorPosition().getValueAsDouble() - setpoint) < armTol*360.0/gearRatio;
      } else if (!armMotorLFailure) {
        armMotorL.setControl(new MotionMagicDutyCycle(setpoint).withSlot(1).withEnableFOC(true));
        atSetpoint = Math.abs(armMotorL.getRotorPosition().getValueAsDouble() - setpoint) < armTol*360.0/gearRatio;
      } else if (!armMotorRFailure) {
        armMotorR.setControl(new MotionMagicDutyCycle(setpoint).withSlot(1).withEnableFOC(true));
        atSetpoint = Math.abs(armMotorR.getRotorPosition().getValueAsDouble() - setpoint) < armTol*360.0/gearRatio;
      }
    }
  }

  // Returns true if the arm currently at the angle specified by armSetpoint, within the tolerance specified by armTol.
  public boolean atSetpoint() {
    return atSetpoint;
  }

  // Changes the angle that the arm will move to. Units: degrees
  public void updateSetpoint(double _armSetpoint) {
    if (_armSetpoint > highLimit) {
      armSetpoint = highLimit;
    } else if (_armSetpoint < lowLimit) {
      armSetpoint = lowLimit;
    } else {
      armSetpoint = _armSetpoint;
    }
  }

  // Returns the position of the arm in degrees.
  public double getArmEncoder() {
    double encoderValue = armEncoderZero - armEncoder.getAbsolutePosition();
    return encoderValue*360.0; 
  }

  public void setManualPower(double _manualPower) {
    manualPower = _manualPower;
  }

  // Toggles whether the arm is under manual control. Useful in the case of motor issues.
  public void toggleManualControl() {
    manualControl = !manualControl;
  }

  // Returns true if the arm is under manual control, and false if it is automated.
  public boolean getManualControl() {
    return manualControl;
  }

  // Returns true if either of the motors failed to configure on startup or reboot.
  public boolean getMotorFailure() {
    return armMotorLFailure || armMotorRFailure;
  }

  // Attempts to reboot by reconfiguring the motors. Use if trying to troubleshoot during a match.
  public void reboot() {
    armMotorLFailure = !configMotor(armMotorL, armMotorLFailure, false);
    armMotorRFailure = !configMotor(armMotorR, armMotorRFailure, true);
    manualControl = getMotorFailure();  
    calibrate();
  }

  // Syncs the falcon encoder and arm encoder.
  public void calibrate() {
   if (!getMotorFailure()) {
      armMotorInitialPos = armMotorL.getRotorPosition().getValueAsDouble();
      armEncoderInitialPos = getArmEncoder();
      isCalibrated = true;
    } else if (!armMotorRFailure) {
      armMotorInitialPos = armMotorR.getRotorPosition().getValueAsDouble();
      armEncoderInitialPos = getArmEncoder();
      isCalibrated = true;
    } else {
      isCalibrated = false;
    }
  }

  // Indicates whether the arm was able to calibrate its position on start up.
  public boolean isCalibrated() {
    return isCalibrated;
  }

  // Sends information to the dashboard each period. This is handled automatically by the class.
  public void updateDashboard() {
    SmartDashboard.putBoolean("manualArmControl", manualControl);
    SmartDashboard.putBoolean("armFailure", getMotorFailure());
    SmartDashboard.putBoolean("atArmSetpoint", atSetpoint());
    SmartDashboard.putNumber("armSetpoint", armSetpoint);
    SmartDashboard.putNumber("armAngle", getArmEncoder());
    SmartDashboard.putBoolean("Arm Calibrated", isCalibrated());
  }

  // Sets PID constants, brake mode, inverts, and enforces a 40 A current limit. Returns true if the motor successfully configured.
  private boolean configMotor(TalonFX _motor, boolean motorFailure, boolean isInverted) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = _motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    motorConfigs.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // Inverts the direction of positive motor velocity.

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
    motorConfigs.MotionMagic.MotionMagicAcceleration = 200.0;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    motorConfigs.MotionMagic.MotionMagicJerk = 500.0;
    
    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int motorErrors = 0;
    while (motorConfigurator.apply(motorConfigs, 0.03) != StatusCode.OK) {
        motorErrors++;
      motorFailure = motorErrors > maxMotorFailures;
      if (motorFailure) {
        disableMotor(_motor);
        return false;
      }
    }
    return true;
  }   

  // Attempts to sets the motor to coast mode with 0 output. Used in the case of a motor failure.
  private void disableMotor(TalonFX motor) {
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigurator.refresh(motorConfigs);
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfigurator.apply(motorConfigs);

    motor.setControl(new DutyCycleOut(0));
  }
}