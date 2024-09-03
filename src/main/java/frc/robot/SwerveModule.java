package frc.robot;

import com.ctre.phoenix6.StatusCode;
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
  private static final double correctionFactor = 0.98; // Factor that corrects for real-world deviations from the odometry calculated position of the robot. These can be caused by things like tread wear. Set this value to 1, then make the robot follow a 1 meter path in auto. Set this value to the distance the robot actually traveled.
  private static final double wheelCirc = 4.0*0.0254*Math.PI; // Circumference of the wheel. Unit: meters
  private static final double turnGearRatio = 150.0/7.0; // Turn motor rotor rotations per turn rotation of the swerve wheel.
  private static final double driveGearRatio = 300.0/49.0; // Drive motor rotor rotations per drive rotation of the swerve wheel.
  private final CANcoder wheelEncoder; // The wheel encoder connected the the DIO port
  private final double wheelEncoderZero; // The reading of the wheel encoder when the wheel is pointed forwards. 
  private final TalonFX driveMotor; // The Falcon 500 motor that controls the driving of the swerve module.
  private final TalonFX turnMotor; // The Falcon 500 motor that controls the turning of the swerve module.
  private double turnMotorInitialPos = 0.0; // The turn motor position on start up in falcon rotations.
  private double driveMotorInitialPos = 0.0; // The drive motor position on start up in falcon rotations.
  private double wheelInitialPos = 0.0; // The wheel encoder position on start up in degrees.
  private double angleSetpoint = 0.0; // The last calculated turn setpoint of the swerve wheel in degrees. Not bounded within 180/-180.
  private boolean driveMotorFailure = false; // Whether the drive motor has failed to configure correctly.
  private boolean turnMotorFailure = false; // Whether the turn motor has failed to configure correctly.
  private boolean encoderFailure = false; // Whether the wheel encoder failed to initialize. 

  public SwerveModule(int turnID, int driveID, int encoderID, boolean invertDrive, double _wheelEncoderZero, String canbus) {
    wheelEncoderZero = _wheelEncoderZero;
    wheelEncoder = new CANcoder(encoderID, canbus);
    encoderFailure = configEncoder(wheelEncoder, 3);
    driveMotor = new TalonFX(driveID, canbus);
    turnMotor = new TalonFX(turnID, canbus);
    driveMotorFailure = !configDriveMotor(driveMotor, invertDrive, 60.0, 3);
    turnMotorFailure = !configTurnMotor(turnMotor, true, 60.0, 3);
    turnMotorInitialPos = turnMotor.getRotorPosition().waitForUpdate(1.0).getValueAsDouble();
    driveMotorInitialPos = driveMotor.getRotorPosition().waitForUpdate(1.0).getValueAsDouble();
    wheelInitialPos = getWheelEncoderAngle();
    angleSetpoint = getTurnMotorAngle();
  }

  // Sets the swerve module to the given state (velocity and angle).
  public void setSMS(SwerveModuleState desiredState) {
    double goalAngleFor = desiredState.angle.getDegrees();
    double goalAngleRev = goalAngleFor > 0.0 ? goalAngleFor - 180.0 : goalAngleFor + 180.0; // Instead of rotating to the input angle, the swerve module can rotate to a position 180 degrees off and reverse the input velocity to achieve the same result.
    double angleSetpointMod360 = angleSetpoint - Math.round(angleSetpoint/360.0)*360.0; // Transforms the angle setpoint to a value that is between -180 and 180 degrees. 
    double[] angleDists = {Math.abs(angleSetpointMod360 - goalAngleFor), 360.0 - Math.abs(angleSetpointMod360 - goalAngleFor), 
      Math.abs(angleSetpointMod360 - goalAngleRev), 360.0 - Math.abs(angleSetpointMod360 - goalAngleRev)}; // Calculates the 4 possible angluar distances to the forwards and reverse goals from the current angle.

    // Finds the minimum angular distance of the 4 options available. 
    int minIndex = 0;
    double minDist = angleDists[0];
    for (int currIndex = 1; currIndex < angleDists.length; currIndex++) {
      if (angleDists[currIndex] < minDist) {
        minDist = angleDists[currIndex];
        minIndex = currIndex;
      }
    }

    // Sets the output angle based on the minimum angular distance. Also reverses the velocity of the swerve module if the minimum distance is based on a reversed angle. 
    double outputAngle = angleSetpoint;
    boolean reverseVel = false;
    if (minIndex == 0) { // Forward angle, does not cross 180/-180.
      outputAngle = goalAngleFor > angleSetpointMod360 ? angleSetpoint + minDist : angleSetpoint - minDist;
    } else if (minIndex == 1) { // Forward angle, crosses 180/-180.
      outputAngle = goalAngleFor > angleSetpointMod360 ? angleSetpoint - minDist : angleSetpoint + minDist;
    } else if (minIndex == 2) { // Reverse angle, does not cross 180/-180
      outputAngle = goalAngleRev > angleSetpointMod360 ? angleSetpoint + minDist : angleSetpoint - minDist;
      reverseVel = true;
    } else { // Reverse angle, crosses 180/-180
      outputAngle = goalAngleRev > angleSetpointMod360 ? angleSetpoint - minDist : angleSetpoint + minDist;
      reverseVel = true;
    }
    double goalVel = reverseVel ? -desiredState.speedMetersPerSecond : desiredState.speedMetersPerSecond;

    setAngle(outputAngle);
    setVel(goalVel);
    angleSetpoint = outputAngle;
  }
  
  // Returns the velocity and angle of the module.
  public SwerveModuleState getSMS() {
    return new SwerveModuleState(getDriveMotorVel(), Rotation2d.fromDegrees(getTurnMotorAngle()));
  }
  
  // Returns the postion and angle of the module.
  public SwerveModulePosition getSMP() {
    return new SwerveModulePosition(getDriveMotorPos(), Rotation2d.fromDegrees(getTurnMotorAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  public double getDriveMotorVel() {
    return driveMotor.getVelocity().getValueAsDouble()*wheelCirc*correctionFactor/driveGearRatio;
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getDriveMotorPos() {
    return (driveMotor.getRotorPosition().getValueAsDouble()-driveMotorInitialPos)*wheelCirc*correctionFactor/driveGearRatio;
  }
  
  // Returns the angle of the wheel in degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). Can return values outside of -180 to 180, corresponding to multiple rotations of the swerve wheel.
  public double getTurnMotorAngle() {
    return (turnMotor.getRotorPosition().getValueAsDouble()-turnMotorInitialPos)*360.0/turnGearRatio+wheelInitialPos;
  }
  
  // Returns the raw value of the wheel encoder. Range: -180 to 180 degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y).
  public double getWheelEncoderAngle() {
    double wheelAngle = wheelEncoder.getAbsolutePosition().getValueAsDouble()*360.0 - wheelEncoderZero;
    if (wheelAngle > 180.0) {
      wheelAngle = wheelAngle - 360.0;
    } else if (wheelAngle < -180.0) {
      wheelAngle = wheelAngle + 360.0;
    }
    return wheelAngle;
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
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentThreshold = currentLimit;
    motorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;

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
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentThreshold = currentLimit;
    motorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;

    // Setting Motion Magic parameters
    motorConfigs.Slot0.kP = 17.14;
    motorConfigs.Slot0.kI = 0.1307;
    motorConfigs.Slot0.kD = 0.00028;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0;

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
  private boolean configEncoder(CANcoder encoder, int maxEncoderErrors) {
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