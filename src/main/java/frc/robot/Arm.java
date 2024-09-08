package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
  private final double armTol = 1.0; // The acceptable error in the angle of the arm in degrees.
  private final double gearRatio = 288.0; // 72:12 chain. 3:1, 4:1, and 4:1 stacked planetaries.
  private final double lowLimit = -6.0; // The lower limit of the arm in degrees.
  private final double highLimit = 75.0; // The higher limit of the arm in degrees.
  private final TalonFX armMotorLeft = new TalonFX(12, "canivore"); // One of the motors that controls the arm.
  private final TalonFX armMotorRight = new TalonFX(11, "canivore"); // One of the motors that controls the arm.
  private final DutyCycleEncoder armEncoderLeft = new DutyCycleEncoder(0); // Keeps track of the angle of the arm.
  private final DutyCycleEncoder armEncoderRight = new DutyCycleEncoder(9); // Keeps track of the angle of the arm.
  private boolean armMotorLeftFailure = false; // Indicates whether the left motor failed to configure on startup.
  private boolean armMotorRightFailure = false; // Indicates whether the right motor failed to configure on startup.
  private double armEncoderLeftZero = 0.691; // The reading of the left encoder in rotations when the arm is at 0 degrees.
  private double armEncoderRightZero = 0.659; // The reading of the right encoder in rotations when the arm is at 0 degrees.
  private double armMotorLeftInitialPos = 0.0; // The position of the left arm motor on startup in falcon rotations.
  private double armEncoderInitialPos = 0.0; // The position of the arm encoder on startup in degrees, with a zero offset applied.
  private double setpoint = 75.0; // The last requested setpoint of the arm in degrees. 0 degrees is horizontal and 90 degrees is vertical. 

  public Arm() {
    armMotorLeftFailure = !configArmMotor(armMotorLeft, false, 60.0, 3);
    armMotorRightFailure = !configArmMotor(armMotorRight, true, 60.0, 3);
    armMotorLeftInitialPos = armMotorLeft.getRotorPosition().getValueAsDouble();
    armEncoderInitialPos = getEncoderAverage();
    setpoint = armEncoderInitialPos;
  }

  // Should be called once teleopPeriodic() and autoPeriodic() sections of the main robot code. Neccesary for the class to function.
  public void periodic() {
    double motorSetpoint = armMotorLeftInitialPos + (setpoint-armEncoderInitialPos)*gearRatio/360.0;
    armMotorLeft.setControl(new MotionMagicDutyCycle(motorSetpoint).withSlot(0).withEnableFOC(true));
    armMotorRight.setControl(new Follower(12, true));
  }

  // Returns true if the arm currently at the angle specified by armSetpoint, within the tolerance specified by armTol.
  public boolean atSetpoint() {
    double motorSetpoint = armMotorLeftInitialPos + (setpoint-armEncoderInitialPos)*gearRatio/360.0;
    return Math.abs(armMotorLeft.getRotorPosition().getValueAsDouble() - motorSetpoint) < armTol*360.0/gearRatio;
  }

  // Changes the angle that the arm will move to. Units: degrees
  public void updateSetpoint(double _armSetpoint) {
    if (_armSetpoint > highLimit) {
      setpoint = highLimit;
    } else if (_armSetpoint < lowLimit) {
      setpoint = lowLimit;
    } else {
      setpoint = _armSetpoint;
    }
  }

  // Returns the current setpoint of the arm in degrees
  public double getSetpoint() {
    return setpoint;
  }

  // Returns the position of the arm in degrees, measured by the left encoder.
  public double getEncoderLeft() {
    return (armEncoderLeftZero - armEncoderLeft.getAbsolutePosition())*360.0; 
  }

  // Returns the position of the arm in degrees, measured by the right encoder.
    public double getEncoderRight() {
    return (armEncoderRightZero - armEncoderRight.getAbsolutePosition())*360.0; 
  }

  // Returns the position of the arm in degrees, measured by the average of both encoders.
  public double getEncoderAverage() {
    return (getEncoderLeft() + getEncoderRight())/2.0;
  }

  // Returns true if the left arm motor failed to configure on startup.
  public boolean getLeftMotorFailure() {
    return armMotorLeftFailure;
  }

  // Returns true if the right arm motor failed to configure on startup.
  public boolean getRightMotorFailure() {
    return armMotorRightFailure;
  }

  // Sends information to the dashboard each period. This is handled automatically by the class.
  public void updateDash() {
    SmartDashboard.putBoolean("Arm Left Motor Failure", getLeftMotorFailure());
    SmartDashboard.putBoolean("Arm Right Motor Failure", getLeftMotorFailure());
    SmartDashboard.putBoolean("Arm atSetpoint", atSetpoint());
    SmartDashboard.putNumber("Arm Setpoint", getSetpoint());
    SmartDashboard.putNumber("Arm Left Encoder", getEncoderLeft());
    SmartDashboard.putNumber("Arm Right Encoder", getEncoderRight());
    SmartDashboard.putNumber("Arm Average Encoder", getEncoderAverage());
  }

  // Attempts to configure the arm motor. Sets inverts, neutral mode, and PID constants. Returns true if the motor successfully configued.
  private boolean configArmMotor(TalonFX motor, boolean invert, double currentLimit, int maxMotorErrors) {
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
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 80.0;
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