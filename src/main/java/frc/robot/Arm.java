package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
  private final TalonFX armMotor1 = new TalonFX(14); // One of the motors that controls the arm.
  private final double motorCurrentLimit = 40.0; // Motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 3; // The number of times a motor will attempt to reconfigure before declaring a failure and putting the device into a manual state.

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean armMotor1Failure = false; 

  private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(2); // Keeps track of the angle of the arm.
  private double armEncoderZero = 0.335; // The initial arm position reading of the encoder in rotations.
  private double armSetpoint = 90.0; // The last requested setpoint of the arm in degrees. 0 degrees is horizontal and 90 degrees is vertical. 
  private final double armTol = 1.0; // The acceptable error in the angle of the arm in degrees.
  private final double gearRatio = 288.0; // 72:12 chain. 3:1, 4:1, and 4:1 stacked planetaries.
  private final double lowLimit = -3.0; // The lower limit of the arm in degrees.
  private final double highLimit = 90.0; // The higher limit of the arm in degrees.

  private boolean manualControl = false; // Indicates whether the arm is under manual control. This can happen if there is a motor failure, or if the operator requests it via setManualControl().
  private double manualPower = 0.0; // Stores the desired output of the arm motor when it is under manual control.

  private double armMotor1InitialPos = 0.0; // The position of the arm motor on boot() in falcon rotations.
  private double armEncoderInitialPos = 0.0; // The position of the arm encoder on boot() in degrees, with a zero offset applied.
  private final Timer calibrationTimer = new Timer(); // Keeps track of how long it has been since the arm position was calibrated to the arm encoder.
  private final double calibrationInterval = 1.0; // How often the arm should be calibrated in seconds. Shorter times lead to more oscilation.

  public Arm() {
    reboot();
  }
  
  // Should be called in teleopInit() and autoInit(). Neccesary for proper function of the arm.
  public void init() {
    calibrate();
  }

  // Should be called once teleopPeriodic() and autoPeriodic() sections of the main robot code. Neccesary for the class to function.
  public void periodic() {
    updateDashboard();
    if (calibrationTimer.get() > calibrationInterval) {
      calibrate();
    }
    if (manualControl) {
      if (!getMotorFailure()) {
        armMotor1.setControl(new DutyCycleOut(manualPower).withEnableFOC(true));
      }
    } else {
      manualPower = 0.0;
      double setpoint = armMotor1InitialPos + (armSetpoint-armEncoderInitialPos)*gearRatio/360.0;
      armMotor1.setControl(new MotionMagicDutyCycle(setpoint).withSlot(1).withEnableFOC(true));
    }
  }

  // Returns true if the arm currently at the angle specified by armSetpoint, within the tolerance specified by armTol.
  public boolean atSetpoint() {
    return Math.abs(getArmEncoder() - armSetpoint) < armTol;
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
    double encoderValue = armEncoder.getAbsolutePosition()-armEncoderZero;
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
    return armMotor1Failure;
  }

  // Attempts to reboot by reconfiguring the motors. Use if trying to troubleshoot during a match.
  public void reboot() {
    armMotor1Failure = !configMotor(armMotor1, armMotor1Failure, false);
    manualControl = getMotorFailure();  
    init();
  }

  // Syncs the falcon encoder and arm encoder.
  private void calibrate() {
   if (!armMotor1Failure) {
      armMotor1InitialPos = armMotor1.getRotorPosition().getValueAsDouble();
      armEncoderInitialPos = getArmEncoder();
      calibrationTimer.restart();
    }
  }

  // Sends information to the dashboard each period. This is handled automatically by the class.
  private void updateDashboard() {
    SmartDashboard.putBoolean("manualArmControl", manualControl);
    SmartDashboard.putBoolean("armFailure", getMotorFailure());
    SmartDashboard.putBoolean("atArmSetpoint", atSetpoint());
    SmartDashboard.putNumber("armSetpoint", armSetpoint);
    SmartDashboard.putNumber("armAngle", getArmEncoder());
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