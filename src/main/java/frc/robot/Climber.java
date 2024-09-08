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
  private final TalonFX leftClimbMotor = new TalonFX(10, "canivore"); // The Falcon 500 motor that controls the left climber.
  private final TalonFX rightClimbMotor = new TalonFX(9, "canivore"); // The other Falcon 500 motor that controls the right climber.
  private final DigitalInput leftLimitSensor = new DigitalInput(2); // Hall effect sensor that detects whether a magnet is present. Triggered when the climber is bottomed out.
  private final DigitalInput rightLimitSensor = new DigitalInput(1); // Hall effect sensor that detects whether a magnet is present. Triggered when the climber is bottomed out.
  private final double rotationsToTop = 180.0; // The approximate number of rotations between the bottom and top of the climber's useful range of motion.
  private boolean leftClimbMotorFailure = false; // Indicates whether the motor failed to configure on startup.
  private boolean rightClimbMotorFailure = false; // Indicates whether the motor failed to configure on startup.
  private double leftClimbMotorZero = 0.0; // The rotor position that corresponds to bottoming out the left climber.
  private double rightClimbMotorZero = 0.0; // The rotor position that corresponds to bottoming out the right climber.
  private boolean limitSensorDetected = false; // Indicates whether the climber detected both limit sensors on startup. The climber is locked out if both sensors are not detected.
  private boolean userLockout = true; // Prevents the user from moving the climber if true. Prevents accidental collisions between the arm and the climber.

  public Climber() {
    leftClimbMotorFailure = !configClimbMotor(leftClimbMotor, true, 60.0, 3);
    rightClimbMotorFailure = !configClimbMotor(rightClimbMotor, false, 60.0, 3);
    leftClimbMotorZero = leftClimbMotor.getRotorPosition().getValueAsDouble();
    rightClimbMotorZero = rightClimbMotor.getRotorPosition().getValueAsDouble();
    limitSensorDetected = getLeftLimitSensor() && getRightLimitSensor();
  }

  // Should be called during teleopInit() and autoInit()
  public void init() {
    leftClimbMotorZero = leftClimbMotor.getRotorPosition().getValueAsDouble();
    rightClimbMotorZero = rightClimbMotor.getRotorPosition().getValueAsDouble();
    limitSensorDetected = getLeftLimitSensor() && getRightLimitSensor();
    userLockout = true;
  }

  // Sets the output of each climb motor. the ClimbPower inputs can range from -1 to 1. -1 corresponds to full power down and +1 is full power up.
  public void setManual(double leftClimbPower, double rightClimbPower) {
    if (!userLockout && limitSensorDetected) {
      if (getLeftLimitSensor() && leftClimbPower < 0.0) {
        leftClimbPower = 0.0;
      }
      if (limitSensorDetected && getLeftMotorPosition() > leftClimbMotorZero + rotationsToTop && leftClimbPower > 0.0) {
        leftClimbPower = 0.0;
      }
      leftClimbMotor.setControl(new DutyCycleOut(leftClimbPower).withEnableFOC(true));

      if (getRightLimitSensor() && rightClimbPower < 0.0) {
        rightClimbPower = 0.0;
      }
      if (limitSensorDetected && getRightMotorPosition() > rightClimbMotorZero + rotationsToTop && rightClimbPower > 0.0) {
        rightClimbPower = 0.0;
      }
      rightClimbMotor.setControl(new DutyCycleOut(rightClimbPower).withEnableFOC(true));
    }
  }

  // Slowly lowers both climbers until both limit sensors are triggered. Does not require the userLockout to be disabled. Should not be used during a match, only between matches.
  public void resetClimbers() {
    double leftClimbPower = -0.3;
    double rightClimbPower = -0.3;
    if(getLeftLimitSensor()) {
      leftClimbPower = 0.0;
    }
    if(getRightLimitSensor()) {
      rightClimbPower = 0.0;
    }
    leftClimbMotor.setControl(new DutyCycleOut(leftClimbPower).withEnableFOC(true));
    rightClimbMotor.setControl(new DutyCycleOut(rightClimbPower).withEnableFOC(true));
  }

  // Moves the climbers to the bottom position.
  public void setToBottom() {
    if (!userLockout && limitSensorDetected) {
      leftClimbMotor.setControl(new MotionMagicDutyCycle(leftClimbMotorZero).withEnableFOC(true));
      rightClimbMotor.setControl(new MotionMagicDutyCycle(rightClimbMotorZero).withEnableFOC(true));
    }
  }

  // Moves the climbers to the top position.
  public void setToTop() {
    if (!userLockout && limitSensorDetected) {
      leftClimbMotor.setControl(new MotionMagicDutyCycle(leftClimbMotorZero + rotationsToTop).withEnableFOC(true));
      rightClimbMotor.setControl(new MotionMagicDutyCycle(rightClimbMotorZero + rotationsToTop).withEnableFOC(true));
    }
  }

  // Automatically sets the climbers to a position. Position is between 0 and 1. 0 corresponds to the bottom of the range and 1 corresponds to the top of the range.
  public void set(double desiredPosition) {
    if (!userLockout && limitSensorDetected) {
      if (desiredPosition < 0.0) {
        desiredPosition = 0.0;
      }
      if (desiredPosition > 1.0) {
        desiredPosition = 1.0;
      }
      leftClimbMotor.setControl(new MotionMagicDutyCycle(leftClimbMotorZero + rotationsToTop*desiredPosition).withEnableFOC(true));
      rightClimbMotor.setControl(new MotionMagicDutyCycle(rightClimbMotorZero + rotationsToTop*desiredPosition).withEnableFOC(true));
    }
  }

  // Returns true if the sensor indicates that the left climber is bottomed out.
  public boolean getLeftLimitSensor() {
    return !leftLimitSensor.get();
  }

  // Returns true if the sensor indicates that the right climber is bottomed out.
  public boolean getRightLimitSensor() {
    return !rightLimitSensor.get();
  }
  
  // Allows the user to move the climbers. Should only be called at the end of a match after the arm has been lowered.
  public void disableUserLockout() {
    userLockout = false;
  }

  // Returns true if the user is locked out from using the climbers. 
  public boolean getUserLockout() {
    return userLockout;
  }

  // Returns the rotor position of the left climb motor.
  public double getLeftMotorPosition() {
     return leftClimbMotor.getRotorPosition().getValueAsDouble();
  }

  // Returns the rotor position of the right climb motor.
  public double getRightMotorPosition() {
     return rightClimbMotor.getRotorPosition().getValueAsDouble();
  } 

  // Returns true if both climbers triggered the limit sensors on startup.
  public boolean getLimitSensorDetected() {
    return limitSensorDetected;
  }

  // Returns true if the left motor failed to configure on startup.
  public boolean getLeftClimbMotorFailure() {
    return leftClimbMotorFailure;
  }

  // Returns true if the right motor failed to configure on startup.
  public boolean getRightClimbMotorFailure() {
    return rightClimbMotorFailure;
  }

  // Updates any important values on the dashboard.
  public void updateDash() {
    SmartDashboard.putBoolean("Climber Left Motor Failure", getLeftClimbMotorFailure());
    SmartDashboard.putBoolean("Climber Right Motor Failure", getRightClimbMotorFailure());
    SmartDashboard.putBoolean("Climber Left Limit Sensor", getLeftLimitSensor());
    SmartDashboard.putBoolean("Climber Right Limit Sensor", getRightLimitSensor());
    SmartDashboard.putBoolean("Climber User Lockout", getUserLockout());
    SmartDashboard.putBoolean("Climber Limit Sensor Detected", getLimitSensorDetected());
    SmartDashboard.putNumber("Climber Left Motor Position", getLeftMotorPosition());
    SmartDashboard.putNumber("Climber Right Motor Position", getRightMotorPosition());
  }

  // Attempts to configure the climb motor. Sets inverts, neutral mode, and PID constants. Returns true if the motor successfully configued.
  private boolean configClimbMotor(TalonFX motor, boolean invert, double currentLimit, int maxMotorErrors) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Setting current limits
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentThreshold = currentLimit;
    motorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;

    // Setting Motion Magic parameters
    motorConfigs.Slot0.kP = 0.8;
    motorConfigs.Slot0.kI = 2.0;
    motorConfigs.Slot0.kD = 0.006;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 200.0;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    motorConfigs.MotionMagic.MotionMagicJerk = 500.0;

    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int motorErrors = 0;
    boolean motorFailure = false;
    while (motorConfigurator.apply(motorConfigs, 0.03) != StatusCode.OK) {
      motorErrors++;
      motorFailure = motorErrors > maxMotorErrors;
      if (motorFailure) {
        return false;
      }
    }  
    return true;
  }
}