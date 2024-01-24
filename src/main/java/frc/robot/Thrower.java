package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Thrower {
  private final double spinUpDelay = 0.5; // The amount of time in seconds that the thrower motor is allowed to stay at 100% power without attaining the commanded flywheel velocity before the note is thrown. This value should correspond to the amount of time the thrower motor takes to spin up to full speed.
  private final double loadDelay = 0.3; // The amount of the time in seconds that the thrower will continute to intake after detecting a note.
  private final double unloadDelay = 0.6; // The amount of time in seconds that the thrower will continue to throw after it has detected that the note is no longer present.
  private final double intakeVel = 2.0; // The number of rotations per second that the motors will spin in reverse when intaking a not

  private final double allowableFlywheelVelError = 2.0; // The number of rotations per second of error in the flywheel velocity that is acceptable before a note begins to be launched.
  private final double throwMotorCurrentLimit = 40.0; // Throw motor current limit in amps. Should be based on the breaker used in the PDP.
  private final double indexMotorCurrentLimit = 20.0; // Index motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 20; // The number of times a motor will attempt to reconfigure on start up.

  private final TalonFX throwMotor = new TalonFX(10); // The motor running the outer flywheel (thrower).
  private final TalonFX indexMotor = new TalonFX(9); // The motor running the inner flywheel (indexer).

  // Initializes the proximity sensors. These return false if an object is detected and true if no object is detected.
  private final DigitalInput sensor0 = new DigitalInput(0);
  private final DigitalInput sensor1 = new DigitalInput(1);

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean throwMotorFailure = false; 
  private boolean indexMotorFailure = false;

  private final Timer noteLoadedTimer = new Timer(); // Stores the number of seconds since the proximity sensors detected a note.
  private final Timer noteUnloadedTimer = new Timer(); // Stores the number of seconds since the proximity sensor detected the abscense of a note.
  private final Timer spinUpTimer = new Timer(); // Keeps track of how long the thrower motor has spent at 100% power. 
  private boolean pastSensor = true; // Stores the previous sensor reading. Used to check for changes in the sensor reading.
  private boolean currSensor = true; // Stores the current sensor reading. 
  private boolean isSpunUp = false; // Returns true if the thrower flywheel is at speed, or at least 1 second has passed since the thrower flywheel attempted to get to speed (in case the flywheel cannot reach the demanded angular velocity).
  private boolean throwCommanded = false; // Returns true if a throw command was recieved, but not yet executed.
  private double flywheelVel = 120.0; // The last demanded flywheel velocity in rotations per second. 120 rps is roughly the max speed of a free spining Falcon.

  public Thrower() {
    configIndexMotor();
    configThrowMotor();
    noteLoadedTimer.start();
    noteUnloadedTimer.start();
    spinUpTimer.start();
  }

  // Resets thrower's state upon start up. Should be called in teleopInit() and autoInit()
  public void init() {
    spinUpTimer.restart();
    isSpunUp = false;
    throwCommanded = false;
    pastSensor = getSensor();
    currSensor = pastSensor;
  }

  // This function should be called periodically during teleop and auto. It updates the internal state of the thrower.
  public void periodic() {
    // Updates the proximity sensor and checks for changes. The corresponding timers are restarted if a change is detected, indicating that a note was loaded or unloaded.
    currSensor = getSensor();
    if (currSensor && !pastSensor) {
      noteLoadedTimer.restart();
    } else if (!currSensor && pastSensor) {
      noteUnloadedTimer.restart();
    }
    pastSensor = currSensor;
    
    // Restarts the spinUpTimer if the thrower motor is not running at 100% power.
    if (throwMotor.getDutyCycle().getValueAsDouble() != 1.0) {
      spinUpTimer.restart();
    }

    isSpunUp = Math.abs(throwMotor.getVelocity().getValueAsDouble() - flywheelVel) < allowableFlywheelVelError || spinUpTimer.get() > spinUpDelay; // Checks to see whether the flywheel has reached the target angular velocity, or if it has held 100% power (duty cycle) for >0.5s.
    throwCommanded = throwCommanded && (currSensor || noteUnloadedTimer.get() < unloadDelay); // Reverts throwCommanded to false if a note is not detected and 0.6s has passed.

    // Determines the state (throw, intake, spin up) of the thrower.
    if (throwCommanded && isSpunUp) { // Throws a note if a throw is commanded and the thrower motor is spun up.
      throwNote();
    } else if (!currSensor || noteLoadedTimer.get() < loadDelay) { // Intakes a note if one is not detected or if less than 0.3s has passed since a note was detected.
      intakeNote();
    } else { // Spins up the thrower motor if neither of the previous conditions is met.
      spinUp();
    }
  }

  // Returns true if either the thrower motor or the index motor failed to configure on start up.
  public boolean motorFailure() {
    return throwMotorFailure || indexMotorFailure;
  }

  // Call when a note should be thrown. This will spin up the flywheel and release the note when the flywheel is at speed. flywheelVel is in rotations per second.
  public void commandThrow(double _flywheelVel) {
    flywheelVel = _flywheelVel;
    throwCommanded = true;
  }

  // Call to set the flywheel velocity to a different value without throwing a note. flywheelVel is in rotations per second.
  public void setFlywheelVel(double _flywheelVel) {
    flywheelVel = _flywheelVel;
  }

  // Returns true if the thrower is in the process of spinning up and throwing a note.
  public boolean isThrowing() {
    return throwCommanded;
  }

  // Returns true if either proximity sensor on the thrower is triggered.
  public boolean getSensor() {
    return !sensor0.get() || !sensor1.get();
  }
  
  // Throws the note.
  private void throwNote() {
    throwMotor.setControl(new VelocityDutyCycle(flywheelVel));
    indexMotor.setControl(new VelocityDutyCycle(flywheelVel));
  }

  // Spins up the thrower flywheel while holding the note.
  private void spinUp() {
    throwMotor.setControl(new VelocityDutyCycle(flywheelVel));
    indexMotor.setControl(new VelocityDutyCycle(0.0));
  }

  // Runs both motors backwards to load a note.
  private void intakeNote() {
    throwMotor.setControl(new VelocityDutyCycle(-intakeVel));
    indexMotor.setControl(new VelocityDutyCycle(-intakeVel));
  }

  // Sets velocity PIDV constants, brake mode, and enforces a 40 A current limit.
  private void configThrowMotor() {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator throwMotorConfigurator = throwMotor.getConfigurator();
    TalonFXConfiguration throwMotorConfigs = new TalonFXConfiguration();

    throwMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    throwMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverts the direction of positive motor velocity.

    // Setting current limits
    throwMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    throwMotorConfigs.CurrentLimits.SupplyCurrentLimit = throwMotorCurrentLimit;
    throwMotorConfigs.CurrentLimits.SupplyCurrentThreshold = throwMotorCurrentLimit;
    throwMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;

    // Velocity PIDV constants for reaching flywheel velocities
    throwMotorConfigs.Slot0.kP = 0.008;
    throwMotorConfigs.Slot0.kI = 0.06;
    throwMotorConfigs.Slot0.kD = 0.0002;
    throwMotorConfigs.Slot0.kV = 0.009;
    
    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int throwMotorErrors = 0;
    while (throwMotorConfigurator.apply(throwMotorConfigs, 0.03) != StatusCode.OK) {
        throwMotorErrors++;
      throwMotorFailure = throwMotorErrors > maxMotorFailures;
      if (throwMotorFailure) {
        break;
      }
    }
  }

  // Sets velocity PIDV constants, brake mode, and enforces a 20 A current limit.
  private void configIndexMotor() {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator indexMotorConfigurator = indexMotor.getConfigurator();
    TalonFXConfiguration indexMotorConfigs = new TalonFXConfiguration();

    indexMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    indexMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverts the direction of positive motor velocity.

    // Setting current limits
    indexMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMotorConfigs.CurrentLimits.SupplyCurrentLimit = indexMotorCurrentLimit;
    indexMotorConfigs.CurrentLimits.SupplyCurrentThreshold = indexMotorCurrentLimit;
    indexMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;

    // Velocity PIDV constants for reaching flywheel velocities
    indexMotorConfigs.Slot0.kP = 0.008;
    indexMotorConfigs.Slot0.kI = 0.06;
    indexMotorConfigs.Slot0.kD = 0.0002;
    indexMotorConfigs.Slot0.kV = 0.009;
    
    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int indexMotorErrors = 0;
    while (indexMotorConfigurator.apply(indexMotorConfigs, 0.03) != StatusCode.OK) {
      indexMotorErrors++;
      indexMotorFailure = indexMotorErrors > maxMotorFailures;
      if (indexMotorFailure) {
        break;
      }
    }
  }
}