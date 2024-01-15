package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

class SwerveModule {
  private static final double correctionFactor = 0.95; // Factor that corrects for real-world deviations from the odometry calculated position of the robot. These can be caused by things like tread wear. Set this value to 1, then make the robot follow a 1 meter path in auto. Set this value to the distance the robot actually traveled.
  private static final double wheelCirc = 4.0*0.0254*Math.PI; // Circumference of the wheel. Unit: meters
  private static final double turnGearRatio = 150.0/7.0;
  private static final double driveGearRatio = 57.0/7.0;
  private static final double driveMotorCurrentLimit = 40.0; // Single motor current limit in amps.
  private static final double turnMotorCurrentLimit = 20.0; // Single motor current limit in amps.
  private final AnalogEncoder wheelEncoder;
  private final double wheelEncoderZero;
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final boolean invertDrive;
  private double turnMotorInitialPos;
  private double driveMotorInitialPos;
  private double wheelInitialPos;

  // Motor error code tracking variables.
  private final int maxMotorErrors = 20; // The most times a configuration command can be unsuccesfully sent to a motor before a failure is declared and the motor is disabled. 
  private boolean driveMotorFailure = false; // Whether the drive motor has failed to configure correctly.
  private boolean turnMotorFailure = false; // Whether the turn motor has failed to configure correctly.
  private boolean moduleFailure = false; // Whether either the drive motor or the turn motor has failed to configure correctly.
  private boolean moduleDisabled = false; // Whether the module has been disabled by the driver.

  public SwerveModule(int turnID, int driveID, int encoderID, boolean _invertDrive, double _wheelEncoderZero) {
    wheelEncoderZero = _wheelEncoderZero;
    invertDrive = _invertDrive;
    wheelEncoder = new AnalogEncoder(encoderID);
    driveMotor = new TalonFX(driveID);
    turnMotor = new TalonFX(turnID);
    configDriveMotor();
    configTurnMotor();
    moduleDisabled = driveMotorFailure && turnMotorFailure;
    moduleFailure = driveMotorFailure || turnMotorFailure;
  }

  // Sets the swerve module to the given state (velocity and angle).
  public void setSMS(SwerveModuleState desiredState) {
    double goalAngleFor = desiredState.angle.getDegrees();
    double goalAngleRev = goalAngleFor > 0.0 ? goalAngleFor - 180.0 : goalAngleFor + 180.0; // Instead of rotating to the input angle, the swerve module can rotate to a position 180 degrees off and reverse the input velocity to achieve the same result.
    double currAngle = getAngle();
    double currAngleMod360 = currAngle - Math.round(currAngle/360.0)*360.0; // Limits currAngle between -180 and 180 degrees. 
    double[] AngleDists = {Math.abs(currAngleMod360 - goalAngleFor), 360.0 - Math.abs(currAngleMod360 - goalAngleFor)
      , Math.abs(currAngleMod360 - goalAngleRev), 360.0 - Math.abs(currAngleMod360 - goalAngleRev)}; // Calculates the 4 possible angluar distances to the forwards and reverse goals from the current angle.

    // Finds the minimum angular distance of the 4 options available. 
    int minIndex = 0;
    double minDist = AngleDists[0];
    for (int currIndex = 1; currIndex < AngleDists.length; currIndex++) {
      if (AngleDists[currIndex] < minDist) {
        minDist = AngleDists[currIndex];
        minIndex = currIndex;
      }
    }

    // Sets the output angle based on the minimum angular distance. Also reverses the velocity of the swerve module if the minimum distance is based on a reversed angle. 
    double outputAngle = currAngle;
    boolean reverseVel = false;
    if (minIndex == 0) { // Forward angle, does not cross 180/-180.
      outputAngle = goalAngleFor > currAngleMod360 ? currAngle + minDist : currAngle - minDist;
    } else if (minIndex == 1) { // Forward angle, crosses 180/-180.
      outputAngle = goalAngleFor > currAngleMod360 ? currAngle - minDist : currAngle + minDist;
    } else if (minIndex == 2) { // Reverse angle, does not cross 180/-180
      outputAngle = goalAngleRev > currAngleMod360 ? currAngle + minDist : currAngle - minDist;
      reverseVel = true;
    } else { // Reverse angle, crosses 180/-180
      outputAngle = goalAngleRev > currAngleMod360 ? currAngle - minDist : currAngle + minDist;
      reverseVel = true;
    }
    double goalVel = reverseVel ? -desiredState.speedMetersPerSecond : desiredState.speedMetersPerSecond;

    setAngle(outputAngle);
    setVel(goalVel);
  }
  
  // Returns the velocity and angle of the module.
  public SwerveModuleState getSMS() {
    return new SwerveModuleState(getVel(), Rotation2d.fromDegrees(getAngle()));
  }
  
  // Returns the postion and angle of the module.
  public SwerveModulePosition getSMP() {
    return new SwerveModulePosition(getPos(), Rotation2d.fromDegrees(getAngle()));
  }

  // Returns the velocity of the wheel. Unit: meters per second
  public double getVel() {
    if (!driveMotorFailure && !moduleDisabled) {
      return driveMotor.getVelocity().getValueAsDouble()*wheelCirc*correctionFactor/driveGearRatio;
    } else {
      return 0;
    }
  }

  // Returns total distance the wheel has rotated. Unit: meters
  public double getPos() {
    if (!driveMotorFailure && !moduleDisabled) {
      return (driveMotor.getRotorPosition().getValueAsDouble()-driveMotorInitialPos)*wheelCirc*correctionFactor/driveGearRatio;
    } else {
      return 0;
    }
  }
  
  // Returns the angle of the wheel in degrees. 0 degrees corresponds to facing to the front (+x). 90 degrees in facing left (+y). 
  public double getAngle() {
    if (!turnMotorFailure && !moduleDisabled) {
      return (turnMotor.getRotorPosition().getValueAsDouble()-turnMotorInitialPos)*360.0/turnGearRatio+wheelInitialPos;
    } else {
      return 0;
    }
  }
  
  // Returns the raw value of the wheel encoder. Range: 0-360 degrees.
  public double getWheelEncoder() {
    double wheelAngle = wheelEncoder.getAbsolutePosition()*360.0 - wheelEncoderZero;
    if (wheelAngle > 180.0) {
      wheelAngle = wheelAngle - 360.0;
    } else if (wheelAngle < -180.0) {
      wheelAngle = wheelAngle + 360.0;
    }
    return wheelAngle;
  }
  
  // Sets the velocity of the module. Units: meters per second
  private void setVel(double vel) {
    if (!driveMotorFailure && !moduleDisabled) {
      driveMotor.setControl(new VelocityDutyCycle(vel*driveGearRatio/(wheelCirc*correctionFactor)));
    } else {
      driveMotor.setControl(new DutyCycleOut(0));
    }
  }
  
  // Sets the angle of the module. Units: degrees
  private void setAngle(double angle) {
    if (!turnMotorFailure && !moduleDisabled) {
      turnMotor.setControl(new MotionMagicDutyCycle(((angle-wheelInitialPos)*turnGearRatio)/360.0+turnMotorInitialPos));
    } else {
      turnMotor.setControl(new DutyCycleOut(0));
    }
  }

  // Toggles whether the module is enabled or disabled. Used in the case of an engine failure.
  public void toggleModule() {
    if (moduleDisabled) {
      enableDriveMotor();
      enableTurnMotor();
    } else {
      disableDriveMotor();
      disableTurnMotor();
    }
    moduleDisabled = !moduleDisabled;
    moduleFailure = driveMotorFailure || turnMotorFailure;
  }

  // True if the drive motor failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getDriveMotorFailure() {
    return driveMotorFailure;
  }

  // True if the turn motor failed to respond to configuration commands on startup or reboot. Is a likely indicator of motor or CAN failure.
  public boolean getTurnMotorFailure() {
    return turnMotorFailure;
  }

  // True if the drive motor *or* turn motor failed to configure.
  public boolean getModuleFailure() {
    return moduleFailure;
  }

  // True if the drive motor *and* the turn motor failed to configure on startup, or if the driver disabled the module.
  public boolean getModuleDisabled() {
    return moduleDisabled;
  }

  // The following 2 functions disable the motor in the case of too many CAN errors, or if the driver chooses to disable the module in the case of an engine or mechanical failure.
  private void disableTurnMotor() {
    TalonFXConfigurator turnMotorConfigurator = turnMotor.getConfigurator();
    TalonFXConfiguration turnMotorConfigs = new TalonFXConfiguration();
    turnMotorConfigurator.refresh(turnMotorConfigs);
    turnMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turnMotorConfigurator.apply(turnMotorConfigs);

    turnMotor.setControl(new DutyCycleOut(0));
  }

  private void disableDriveMotor() {
    TalonFXConfigurator driveMotorConfigurator = driveMotor.getConfigurator();
    TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();
    driveMotorConfigurator.refresh(driveMotorConfigs);
    driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveMotorConfigurator.apply(driveMotorConfigs);

    driveMotor.setControl(new DutyCycleOut(0));
  }

  // The following 2 functions re-enable the motor. These should be called if the motors were previously disabled by the driver, or failed to properly initialize at robot start-up. Motors will automatically be re-disabled if they are not able to be configured properly.
  private void enableTurnMotor() {
    turnMotorFailure = false;
    configTurnMotor();
  }

  private void enableDriveMotor() {
    turnMotorFailure = false;
    configDriveMotor();
  }

  private void configDriveMotor() {
    TalonFXConfigurator driveMotorConfigurator = driveMotor.getConfigurator();
    TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

    driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotorConfigs.MotorOutput.Inverted = invertDrive ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = driveMotorCurrentLimit;
    driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = driveMotorCurrentLimit;
    driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;
    driveMotorConfigs.Slot0.kP = 0.008;
    driveMotorConfigs.Slot0.kI = 0.06;
    driveMotorConfigs.Slot0.kD = 0.0002;
    driveMotorConfigs.Slot0.kV = 0.009;

    int driveMotorErrors = 0;
    while (driveMotorConfigurator.apply(driveMotorConfigs, 0.03) != StatusCode.OK) {
      driveMotorErrors++;
      driveMotorFailure = driveMotorErrors > maxMotorErrors;
      if (driveMotorFailure) {
        disableDriveMotor();
        break;
      }
    }

    driveMotorInitialPos = driveMotorFailure ? 0.0 : driveMotor.getRotorPosition().getValueAsDouble();
  }

  private void configTurnMotor() {
    TalonFXConfigurator turnMotorConfigurator = turnMotor.getConfigurator();
    TalonFXConfiguration turnMotorConfigs = new TalonFXConfiguration();

    turnMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turnMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotorConfigs.CurrentLimits.SupplyCurrentLimit = turnMotorCurrentLimit;
    turnMotorConfigs.CurrentLimits.SupplyCurrentThreshold = turnMotorCurrentLimit;
    turnMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;
    turnMotorConfigs.Slot0.kP = 0.8;
    turnMotorConfigs.Slot0.kI = 2.0;
    turnMotorConfigs.Slot0.kD = 0.006;
    turnMotorConfigs.MotionMagic.MotionMagicAcceleration = 500.0;
    turnMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 1000.0;

    int turnMotorErrors = 0;
    while (turnMotorConfigurator.apply(turnMotorConfigs, 0.03) != StatusCode.OK) {
      turnMotorErrors++;
      turnMotorFailure = turnMotorErrors > maxMotorErrors;
      if (turnMotorFailure) {
        disableTurnMotor();
        break;
      }
    }
    
    wheelInitialPos = getWheelEncoder();
    turnMotorInitialPos = turnMotorFailure ? 0.0 : turnMotor.getRotorPosition().getValueAsDouble();
  }
}