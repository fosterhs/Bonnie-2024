package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  public SwerveModule(int turnID, int driveID, int encoderID, boolean invertDrive, double wheelEncoderZero, String canbus) {
    wheelEncoder = new CANcoder(encoderID, canbus);
    configEncoder(wheelEncoder, wheelEncoderZero);
    turnMotor = new TalonFX(turnID, canbus);
    configTurnMotor(turnMotor, true, 40.0);
    driveMotor = new TalonFX(driveID, canbus);
    configDriveMotor(driveMotor, invertDrive, 100.0);
    driveMotor.setPosition(0.0, 0.03);
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, driveMotor.getPosition(), driveMotor.getVelocity(), wheelEncoder.getAbsolutePosition(), wheelEncoder.getVelocity());
  }

  // Sets the swerve module to the given state (velocity and angle).
  public void setSMS(SwerveModuleState desiredState) {
    Rotation2d currentWheelAngle = Rotation2d.fromDegrees(getWheelAngle());
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentWheelAngle); // Minimizes the amount a wheel needs to rotate by inverting the direction of the drive motor in some situations. 
    setAngle(optimizedState.angle.getDegrees());
    setVel(optimizedState.speedMetersPerSecond*optimizedState.angle.minus(currentWheelAngle).getCos()); // Cosine compensation. If a wheel is not at its angular setpoint, its velocity setpoint is reduced.
  }
  
  // Returns the postion and angle of the module.
  public SwerveModulePosition getSMP() {
    return new SwerveModulePosition(getDriveMotorPos(), Rotation2d.fromDegrees(getWheelAngle()));
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getDriveMotorPos() {
    return BaseStatusSignal.getLatencyCompensatedValue(driveMotor.getPosition(), driveMotor.getVelocity(), 0.02)*wheelCirc*correctionFactor/driveGearRatio;
  }
  
  // Returns the raw value of the wheel encoder. Range: -180 to 180 degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). CCW positive coordinate system.
  public double getWheelAngle() {
    return BaseStatusSignal.getLatencyCompensatedValue(wheelEncoder.getAbsolutePosition(), wheelEncoder.getVelocity(), 0.02)*360.0;
  }
  
  // Sets the velocity of the module. Units: meters per second
  private void setVel(double vel) {
    driveMotor.setControl(new VelocityVoltage(vel*driveGearRatio/(wheelCirc*correctionFactor)).withEnableFOC(true));
  }
  
  // Sets the angle of the module. Units: degrees Can accept values outside of -180 to 180, corresponding to multiple rotations of the swerve wheel.
  private void setAngle(double angle) {
    turnMotor.setControl(new MotionMagicTorqueCurrentFOC(angle/360.0));
  }

  // Configures the swerve module's drive motor.
  private void configDriveMotor(TalonFX motor, boolean invert, double currentLimit) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    // Setting Velocity Voltage parameters.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction in the swerve wheel.

    motorConfigurator.apply(motorConfigs, 0.03);
  }

  // Configures the swerve module's turn motor.
  private void configTurnMotor(TalonFX motor, boolean invert, double currentLimit) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    // Setting Motion Magic TorqueFOC parameters
    motorConfigs.Slot0.kP = 800.0; // Units: amperes per 1 swerve wheel rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 swerve wheel rotation * 1 second of error.
    motorConfigs.Slot0.kD = 18.0; // Units: amperes per 1 swerve wheel rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0/turnGearRatio; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0/turnGearRatio; // Units: roations per second.

    // Sets CANcoder parameters
    motorConfigs.Feedback.FeedbackRemoteSensorID = wheelEncoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    motorConfigs.Feedback.RotorToSensorRatio = turnGearRatio;
    motorConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    motorConfigurator.apply(motorConfigs, 0.03);
  }

  // Configures the swerve module's wheel encoder.
  private void configEncoder(CANcoder encoder, double wheelEncoderZero) {
    // Creates a configurator and config object to configure the motor.
    CANcoderConfigurator encoderConfigurator = encoder.getConfigurator();
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();

    // Sets encoder settings such as range, direction and zero. 
    encoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfigs.MagnetSensor.MagnetOffset = wheelEncoderZero;

    encoderConfigurator.apply(encoderConfigs, 0.03);
  }
}