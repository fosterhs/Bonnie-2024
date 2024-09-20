package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveModule {
  private static final double correctionFactor = 0.97; // Factor that corrects for real-world deviations from the odometry calculated position of the robot. These can be caused by things like tread wear. Set this value to 1, then make the robot follow a 1 meter path in auto. Set this value to the distance the robot actually traveled.
  private static final double wheelCirc = 4.0*0.0254*Math.PI; // Circumference of the wheel. Unit: meters
  private static final double turnGearRatio = 150.0/7.0; // Turn motor rotor rotations per turn rotation of the swerve wheel.
  private static final double driveGearRatio = 300.0/49.0; // Drive motor rotor rotations per drive rotation of the swerve wheel.
  private final CANcoder wheelEncoder; // The CANcoder that measures the angle of the swerve wheel.
  private final TalonFX driveMotor; // The Falcon 500 motor that controls the driving of the swerve module.
  private final TalonFX turnMotor; // The Falcon 500 motor that controls the turning of the swerve module.
  private boolean driveMotorFailure = false; // Whether the drive motor has failed to configure correctly.
  private boolean turnMotorFailure = false; // Whether the turn motor has failed to configure correctly.
  private boolean encoderFailure = false; // Whether the wheel encoder has failed to configure correctly.
  public StatusSignal<Double> drivePos; // Stores the current position of the drive motor in shaft revolutions. The value is reset to 0 at startup.
  public StatusSignal<Double> driveVel; // Stores the current velocity of the drive motor in shaft revolutions per second. 
  public StatusSignal<Double> wheelPos; // Stores the current angular position of the swerve wheel in rotations between -0.5 and 0.5.
  public StatusSignal<Double> wheelVel; // Stores the current angular velocity of the swerve wheel in rotations per second. 

  public SwerveModule(int turnID, int driveID, int encoderID, boolean invertDrive, double wheelEncoderZero, String canbus) {
    wheelEncoder = new CANcoder(encoderID, canbus);
    encoderFailure = !configEncoder(wheelEncoder, 3, wheelEncoderZero);
    turnMotor = new TalonFX(turnID, canbus);
    turnMotorFailure = !configTurnMotor(turnMotor, true, 40.0, 3);
    driveMotor = new TalonFX(driveID, canbus);
    driveMotorFailure = !configDriveMotor(driveMotor, invertDrive, 80.0, 3);
    driveMotor.setPosition(0.0, 0.03);
    drivePos = driveMotor.getPosition();
    driveVel = driveMotor.getVelocity();
    wheelPos = wheelEncoder.getAbsolutePosition();
    wheelVel = wheelEncoder.getVelocity();
  }

  // Sets the swerve module to the given state (velocity and angle).
  public void setSMS(SwerveModuleState desiredState) {
    Rotation2d currentWheelAngle = Rotation2d.fromDegrees(getWheelAngle());
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentWheelAngle);
    setAngle(optimizedState.angle.getDegrees());
    setVel(optimizedState.speedMetersPerSecond*optimizedState.angle.minus(currentWheelAngle).getCos()); // Cosine compensation for wheel velocity.
  }
  
  // Returns the velocity and angle of the module.
  public SwerveModuleState getSMS() {
    return new SwerveModuleState(getDriveMotorVel(), Rotation2d.fromDegrees(getWheelAngle()));
  }
  
  // Returns the postion and angle of the module.
  public SwerveModulePosition getSMP() {
    return new SwerveModulePosition(getDriveMotorPos(), Rotation2d.fromDegrees(getWheelAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  public double getDriveMotorVel() {
    return driveVel.getValueAsDouble()*wheelCirc*correctionFactor/driveGearRatio;
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getDriveMotorPos() {
    return BaseStatusSignal.getLatencyCompensatedValue(drivePos, driveVel, 0.02)*wheelCirc*correctionFactor/driveGearRatio;
  }
  
  // Returns the raw value of the wheel encoder. Range: -180 to 180 degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). CCW positive coordinate system.
  public double getWheelAngle() {
    return BaseStatusSignal.getLatencyCompensatedValue(wheelPos, wheelVel, 0.02)*360.0;
  }

  // Returns the velocity of the swerve wheel in degrees/second.
  public double getWheelVel() {
    return wheelVel.getValueAsDouble()*360.0;
  }
  
  // Sets the velocity of the module. Units: meters per second
  private void setVel(double vel) {
    driveMotor.setControl(new VelocityDutyCycle(vel*driveGearRatio/(wheelCirc*correctionFactor)).withEnableFOC(true));
  }
  
  // Sets the angle of the module. Units: degrees Can accept values outside of -180 to 180, corresponding to multiple rotations of the swerve wheel.
  private void setAngle(double angle) {
    turnMotor.setControl(new MotionMagicDutyCycle(angle/360.0).withEnableFOC(true));
  }

  // True if the drive motor failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getDriveMotorFailure() {
    return driveMotorFailure;
  }

  // True if the turn motor failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getTurnMotorFailure() {
    return turnMotorFailure;
  }

  // True if the encoder failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getEncoderFailure() {
    return encoderFailure;
  }

  // Attempts to configure the drive motor. Sets inverts, neutral mode, PID constants, and defines the intiial positions. Returns true if the motor successfully configued.
  private boolean configDriveMotor(TalonFX motor, boolean invert, double currentLimit, int maxMotorErrors) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
    // Setting current limits
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    // Setting PID parameters for velocity control
    motorConfigs.Slot0.kP = 0.008;
    motorConfigs.Slot0.kI = 0.06;
    motorConfigs.Slot0.kD = 0.0002;
    motorConfigs.Slot0.kV = 0.009;

    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorErrors
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

  // Attempts to configure the turn motor. Sets inverts, neutral mode, PID constants, and defines the intiial positions. Returns true if the motor successfully configued.
  private boolean configTurnMotor(TalonFX motor, boolean invert, double currentLimit, int maxMotorErrors) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Setting current limits
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    // Setting Motion Magic parameters
    motorConfigs.Slot0.kP = 17.14;
    motorConfigs.Slot0.kI = 0.1307;
    motorConfigs.Slot0.kD = 0.00028;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 46.67;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 4.667;

    // Sets CANcoder parameters
    motorConfigs.Feedback.FeedbackRemoteSensorID = wheelEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = turnGearRatio;
    motorConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorErrors
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

  // Attempts to configure the encoder. Returns true if the configuration was sucessful 
  private boolean configEncoder(CANcoder encoder, int maxEncoderErrors, double wheelEncoderZero) {
    // Creates a configurator and config object to configure the motor.
    CANcoderConfigurator encoderConfigurator = encoder.getConfigurator();
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();

    // Sets encoder settings such as range, direction and zero. 
    encoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfigs.MagnetSensor.MagnetOffset = wheelEncoderZero;

    // Attempts to repeatedly configure the encoder up to the number of times indicated by maxEncoderErrors
    int encoderErrors = 0;
    boolean encoderFailure = false;
    while (encoderConfigurator.apply(encoderConfigs, 0.03) != StatusCode.OK) {
      encoderErrors++;
      encoderFailure = encoderErrors > maxEncoderErrors;
      if (encoderFailure) {
        return false;
      }
    }
    return true;
  }
}