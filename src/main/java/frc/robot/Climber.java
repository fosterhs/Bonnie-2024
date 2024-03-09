package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
  private final TalonFX leftClimbMotor = new TalonFX(10, "rio"); // The motor that controls the left climber.
  private final TalonFX rightClimbMotor = new TalonFX(9, "rio"); // The other that controls the right climber.
  private final double motorCurrentLimit = 40.0; // Motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 3; // The number of times a motor will attempt to reconfigure before declaring a failure and putting the device into a manual state.

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean leftClimbMotorFailure = false; 
  private boolean rightClimbMotorFailure = false; 

  private final DigitalInput leftLimitSensor = new DigitalInput(2); // Hall effect sensor that detects whether a magnet is present. Triggered when the climber is bottomed out.
  private final DigitalInput rightLimitSensor = new DigitalInput(1); // Hall effect sensor that detects whether a magnet is present. Triggered when the climber is bottomed out.

  private boolean lockout = true; // Prevents the user from moving the climber if true. Prevents accidental collisions between the arm and the climber.
  
  private double leftClimbMotorZero = 0.0; // The rotor position that corresponds to bottoming out the left climber.
  private double rightClimbMotorZero = 0.0; // The rotor position that corresponds to bottoming out the right climber.
  private boolean isCalibrated = false; // Indicates whether the climber was able to calibrate. Calibration requires the limit sensors to be active on boot.
  private final double rotationsToTop = 180.0; // The approximate number of rotations between the bottom and top of the climber's useful range of motion.

  public Climber() {
    reboot();
  }

  public void init() {
    enableLockout();
  }

  // Updates any important values on the dashboard.
  public void updateDashboard() {
    SmartDashboard.putBoolean("Climber Failure", !getMotorFailure());
    SmartDashboard.putBoolean("Left Sensor", getLeftSensor());
    SmartDashboard.putBoolean("Right Sensor", getRightSensor());
    SmartDashboard.putBoolean("Climber Lockout", getLockout());
    SmartDashboard.putBoolean("Climbers Calibrated", isCalibrated());
  }

  // Sets the output of each climb motor. the ClimbPower inputs can range from -1 to 1. -1 corresponds to full power down and +1 is full power up.
  public void setManual(double leftClimbPower, double rightClimbPower) {
    if (!lockout) {
      if (!leftClimbMotorFailure) {
        if (getLeftSensor() && leftClimbPower < 0.0) {
          leftClimbPower = 0.0;
        }
        if (isCalibrated && leftClimbMotor.getRotorPosition().getValueAsDouble() > leftClimbMotorZero + rotationsToTop && leftClimbPower > 0.0) {
          leftClimbPower = 0.0;
        }
        leftClimbMotor.setControl(new DutyCycleOut(leftClimbPower).withEnableFOC(true));
      }
      if (!rightClimbMotorFailure) {
        if (getRightSensor() && rightClimbPower < 0.0) {
          rightClimbPower = 0.0;
        }
        if (isCalibrated && rightClimbMotor.getRotorPosition().getValueAsDouble() > rightClimbMotorZero + rotationsToTop && rightClimbPower > 0.0) {
          rightClimbPower = 0.0;
        }
        rightClimbMotor.setControl(new DutyCycleOut(rightClimbPower).withEnableFOC(true));
      }
    }
  }

  // Moves the climbers to the bottom position.
  public void setToBottom() {
    if (!leftClimbMotorFailure && isCalibrated) {
      leftClimbMotor.setControl(new MotionMagicDutyCycle(leftClimbMotorZero).withEnableFOC(true).withSlot(1));
    }
    if (!rightClimbMotorFailure && isCalibrated) {
      rightClimbMotor.setControl(new MotionMagicDutyCycle(rightClimbMotorZero).withEnableFOC(true).withSlot(1));
    }
  }

  // Moves the climbers to the top position.
  public void setToTop() {
    if (!leftClimbMotorFailure && isCalibrated && !lockout) {
      leftClimbMotor.setControl(new MotionMagicDutyCycle(leftClimbMotorZero + rotationsToTop).withEnableFOC(true).withSlot(1));
    }
    if (!rightClimbMotorFailure && isCalibrated && !lockout) {
      rightClimbMotor.setControl(new MotionMagicDutyCycle(rightClimbMotorZero + rotationsToTop).withEnableFOC(true).withSlot(1));
    }
  }

  // Automatically sets the climbers to a position. Position is between 0 and 1. 0 corresponds to the bottom of the range and 1 corresponds to the top of the range.
  public void set(double desiredPosition) {
    if (desiredPosition < 0.0) {
      desiredPosition = 0.0;
    }
    if (desiredPosition > 1.0) {
      desiredPosition = 1.0;
    }
    if (!leftClimbMotorFailure && isCalibrated && !lockout) {
      leftClimbMotor.setControl(new MotionMagicDutyCycle(leftClimbMotorZero + rotationsToTop*desiredPosition).withEnableFOC(true).withSlot(1));
    }
    if (!rightClimbMotorFailure && isCalibrated && !lockout) {
      rightClimbMotor.setControl(new MotionMagicDutyCycle(rightClimbMotorZero + rotationsToTop*desiredPosition).withEnableFOC(true).withSlot(1));
    }
  }

  // Returns true if the sensor indicates that the climber is bottomed out.
  public boolean getLeftSensor() {
    return !leftLimitSensor.get();
  }

  // Returns true if the sensor indicates that the climber is bottomed out.
  public boolean getRightSensor() {
    return !rightLimitSensor.get();
  }
  
  public void disableLockout() {
    lockout = false;
  }

  public void enableLockout() {
    lockout = true;
  }

  public boolean getLockout() {
    return lockout;
  }
  
  // Attempts to reboot the climber by reconfiguring the motors. Use if trying to troubleshoot a climber failure during a match.
  public void reboot() {
    leftClimbMotorFailure = !configMotor(leftClimbMotor, leftClimbMotorFailure, true);
    rightClimbMotorFailure = !configMotor(rightClimbMotor, rightClimbMotorFailure, false);
    calibrate();
  }

  // Sets the zero-position of the climb motors. Both limit sensors must be active for this function to work.
  public void calibrate() {
    boolean leftCalibrated = false;
    boolean rightCalibrated = false;
    if (!leftClimbMotorFailure && getLeftSensor()) {
      leftClimbMotorZero = leftClimbMotor.getRotorPosition().getValueAsDouble();
      leftCalibrated = true;
    }
    if (!rightClimbMotorFailure && getRightSensor()) {
      rightClimbMotorZero = rightClimbMotor.getRotorPosition().getValueAsDouble();
      rightCalibrated = true;
    }
    isCalibrated = leftCalibrated && rightCalibrated;
  }

  // Returns true if the climbers both were able to sense their positions using the limit sensors when calibrate() was called.
  public boolean isCalibrated() {
    return isCalibrated;
  }

  // Returns true if either of the motors failed to configure on startup or reboot.
  public boolean getMotorFailure() {
    return leftClimbMotorFailure || rightClimbMotorFailure;
  }

  // Sets PID constants, brake mode, inverts, and enforces a 40 A current limit. Returns true if the motor successfully configured.
  private boolean configMotor(TalonFX _motor, boolean motorFailure, boolean inverted) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = _motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    if (inverted) {
      motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverts the direction of positive motor velocity.
    } else {
      motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Inverts the direction of positive motor velocity.
    }

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
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50.0;
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