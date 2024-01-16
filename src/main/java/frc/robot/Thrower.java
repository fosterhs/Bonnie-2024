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
  private final TalonFX throwMotor = new TalonFX(10); // The motor running the outer flywheel (thrower).
  private final TalonFX indexMotor = new TalonFX(9); // The motor running the inner flywheel (indexer).

  // Indicates whether the motor failed to configure on startup. Each motor re-attempts startup configuration up to 20 times before this value is true.
  private boolean throwMotorFailure = false; 
  private boolean indexMotorFailure = false;

  // Initializes the proximity sensors. These return false if an object is detected and true if no object is detected.
  private final DigitalInput sensor0 = new DigitalInput(0);
  private final DigitalInput sensor1 = new DigitalInput(1);

  private final Timer noteLoadedTimer = new Timer(); // Stores the number of seconds since the proximity sensors detected a note.
  private final Timer noteUnloadedTimer = new Timer(); // Stores the number of seconds since the proximity sensor detected the abscense of a note.
  private final Timer spinUpTimer = new Timer(); // Stores the number of seconds since the thrower flywheel began to spin up.

  // Stores the current and immediately previous sensor reading. Used to check for changes in the sensor reading.
  private boolean pastSensor = true;
  private boolean currSensor = true;

  // Tracks whether the thrower flywheel is spinning up. Stores the current and immediately previous values. Used to detect when the thrower flywheel began spinning up and restart the associated timer.
  private boolean pastIsSpinningUp = false;
  private boolean currIsSpinningUp = false;

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

  // Call when a note should be thrown. This will spin up the flywheel and release the note when the flywheel is at speed. flywheelVel is in rotations per second.
  public void commandThrow(double _flywheelVel) {
    flywheelVel = _flywheelVel;
    throwCommanded = true;
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

    isSpunUp = Math.abs(throwMotor.getVelocity().getValueAsDouble() - flywheelVel) < 2.0 || spinUpTimer.get() > 1.0; // Checks to see whether the flywheel has reached the target angular velocity, or if 1 second as elapsed since it began trying.
    currIsSpinningUp = !isSpunUp && currSensor && noteLoadedTimer.get() > 0.3; // The flywheel is spinning up if !isSpinUp and a note is not being intaked.
    if (currIsSpinningUp && !pastIsSpinningUp) {
      spinUpTimer.restart();
    }
    pastIsSpinningUp = currIsSpinningUp;

    throwCommanded = throwCommanded && (getSensor() || noteUnloadedTimer.get() < 0.6); // Reverts throwCommanded to false if a note is not detected and 0.6s has passed.

    if (throwCommanded && isSpunUp) {
      throwNote();
    } else if (!currSensor || noteLoadedTimer.get() < 0.3) { // Intakes a note if one is not detected or if less than 0.3s has passed since a note was detected.
      intakeNote();
    } else {
      spinUp();
    }
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
    throwMotor.setControl(new VelocityDutyCycle(-2.0));
    indexMotor.setControl(new VelocityDutyCycle(-2.0));
  }

  // Sets velocity PIDV constants, brake mode, and enforces a 40 A current limit.
  private void configThrowMotor() {
    TalonFXConfigurator throwMotorConfigurator = throwMotor.getConfigurator();
    TalonFXConfiguration throwMotorConfigs = new TalonFXConfiguration();

    throwMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    throwMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    throwMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    throwMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;
    throwMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 40.0;
    throwMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;
    throwMotorConfigs.Slot0.kP = 0.008;
    throwMotorConfigs.Slot0.kI = 0.06;
    throwMotorConfigs.Slot0.kD = 0.0002;
    throwMotorConfigs.Slot0.kV = 0.009;
    

    int throwMotorErrors = 0;
    while (throwMotorConfigurator.apply(throwMotorConfigs, 0.03) != StatusCode.OK) {
        throwMotorErrors++;
      throwMotorFailure = throwMotorErrors > 20;
      if (throwMotorFailure) {
        break;
      }
    }
  }

  // Sets velocity PIDV constants, brake mode, and enforces a 20 A current limit.
  private void configIndexMotor() {
    TalonFXConfigurator indexMotorConfigurator = indexMotor.getConfigurator();
    TalonFXConfiguration indexMotorConfigs = new TalonFXConfiguration();

    indexMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMotorConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
    indexMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 20.0;
    indexMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.5;
    indexMotorConfigs.Slot0.kP = 0.008;
    indexMotorConfigs.Slot0.kI = 0.06;
    indexMotorConfigs.Slot0.kD = 0.0002;
    indexMotorConfigs.Slot0.kV = 0.009;
    
    int indexMotorErrors = 0;
    while (indexMotorConfigurator.apply(indexMotorConfigs, 0.03) != StatusCode.OK) {
      indexMotorErrors++;
      indexMotorFailure = indexMotorErrors > 20;
      if (indexMotorFailure) {
        break;
      }
    }
  }
}