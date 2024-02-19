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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Thrower {
  private final TalonFX throwMotor1 = new TalonFX(11); // One of the motors running the main flywheel.
  private final TalonFX throwMotor2 = new TalonFX(12); // The other motor running the main flywheel.
  private final TalonFX indexMotor = new TalonFX(13); // The motor running the intake.
  // Initializes the proximity sensors. These return false if an object is detected and true if no object is detected.
  private final DigitalInput sensor1 = new DigitalInput(0); // Sensor closest to the intake. Notes will trigger this sensor first when intaked normally.
  private final DigitalInput sensor2 = new DigitalInput(1); // Sensor closest to the shooter. Notes will trigger this sensor second when intaked normally.
  private final double throwMotorCurrentLimit = 40.0; // Throw motor current limit in amps. Should be based on the breaker used in the PDP.
  private final double indexMotorCurrentLimit = 20.0; // Index motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 3; // The number of times a motor will attempt to reconfigure before declaring a failure and putting the thrower into a manual state.
  private final double intakeVel = 5.0; // The number of rotations per second that the motors will spin in reverse when intaking a note.
  private final double ampVel = 5.0; // The number of rotations per second that the motors will spin forwards when scoing a note in the amp.
  private final double indexOffset = 0.5; // How much the index motor should back off the note after it is detected by the 2nd sensor in falcon rotations.
  private final double indexError = 0.05; // How much allowable error there is in the back off position in falcon rotations.
  private final double allowableFlywheelAccError = 10.0; // The acceleration of the flywheel that is acceptable to be considered spun down in rotations per second squared.
  private final double allowableFlywheelVelError = 2.0; // The number of rotations per second of error in the flywheel velocity that is acceptable before a note begins to be launched.
  private final double spinUpDelay = 1.5; // The amount of time in seconds that the thrower motor is allowed to stay at 100% power without attaining the commanded flywheel velocity before the note is thrown. This value should correspond to the amount of time the thrower motor takes to spin up to full speed.
  private final double throwDelay = 0.7; // The amount of time the flywheel will keep spinning after the note is no longer detected. Ensures the note has exited the flywheel before spinning down.
  private final double ampDelay = 0.7; // The amount of time the thrower will keep backfeeding after the note is no longer detected. Ensures the note has exited before spinning down.
  private final Timer throwTimer = new Timer(); // Keeps track of how long it has been since the note was last detected in the AMP_SCORE state.
  private final Timer ampTimer = new Timer(); // Keeps track of how long it has been since the note was last detected in the THROW state.
  private final Timer spinUpTimer = new Timer(); // Keeps track of how long the thrower has been in the SPIN_UP state.

  // Keeps track of the different states of the thrower.
  private enum State {
    SPIN_DOWN,
    THROW,
    SPIN_UP,
    BACK_UP,
    INTAKE,
    MANUAL,
    DISABLED,
    AMP_SCORE;
  } 
  private State nextState;
  private State lastState;

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean throwMotor1Failure = false; 
  private boolean throwMotor2Failure = false; 
  private boolean indexMotorFailure = false;

  // These variables are used to store the goal position of the motor. Used when the motors need to be moved to a specific position, such as in the BACK_UP state.
  private double indexGoalPos;

  private boolean ampScoreCommanded = false; // Returns true if an amp score command was recieved, but not yet executed.
  private boolean throwCommanded = false; // Returns true if a throw command was recieved, but not yet executed. Reverts to false if a note is not detected.
  private double flywheelVel = 120.0; // The last demanded flywheel velocity in rotations per second. 100 rps is roughly the max speed of a free spining Falcon.

  // These variables store the desired motor velocities which are used and updated when the thrower is in the MANUAL state.
  private boolean manualControl = false;
  private double flywheelPowerManual = 0.0;
  private double indexPowerManual = 0.0;

  public Thrower() {
    reboot();
    init();
  }

  // Should be called once teleopInit() and autoInit() sections of the main robot code. Neccesary for the class to function.
  public void init() {
    lastState = State.DISABLED;
    if (manualControl) {
      nextState = State.MANUAL;
    } else if (getSensor1() || getSensor2()) {
      nextState = State.SPIN_UP;
    } else {
      nextState = State.INTAKE;
    }
    throwTimer.restart();
    spinUpTimer.restart();
    ampTimer.restart();
    throwCommanded = false;
    ampScoreCommanded = false;
  }

  // Should be called once teleopPeriodic() and autoPeriodic() sections of the main robot code. Neccesary for the class to function.
  public void periodic() {
    updateDashboard();
    switch (nextState) {
      case SPIN_DOWN:
        lastState = State.SPIN_DOWN;

        throwMotor2.setControl(new VelocityDutyCycle(0.0).withSlot(0));
        throwMotor1.setControl(new VelocityDutyCycle(0.0).withSlot(0));
        indexMotor.setControl(new VelocityDutyCycle(intakeVel).withSlot(0));

        throwCommanded = false;
        ampScoreCommanded = false;

        if (manualControl) {
          nextState = State.MANUAL;
        } else if (getSensor2()) {
          nextState = State.BACK_UP;
        } else if (isThrowerStopped()) {
          nextState = State.INTAKE;
        } else {
          nextState = State.SPIN_DOWN;
        }
        break;

      case THROW:
        lastState = State.THROW;

        throwMotor2.setControl(new VelocityDutyCycle(flywheelVel).withSlot(0));
        throwMotor1.setControl(new VelocityDutyCycle(flywheelVel).withSlot(0));
        indexMotor.setControl(new VelocityDutyCycle(flywheelVel).withSlot(0));

        if (getSensor1() || getSensor2()) {
          throwTimer.restart();
        }

        if (manualControl) {
          nextState = State.MANUAL;
        } else if (throwTimer.get() > throwDelay) {
          nextState = State.SPIN_DOWN;
        } else {
          nextState = State.THROW;
        }
        break;

      case AMP_SCORE:
        if (lastState != State.AMP_SCORE) {
          ampTimer.restart();
        }
        lastState = State.AMP_SCORE;

        indexMotor.setControl(new VelocityDutyCycle(ampVel).withSlot(0));
        throwMotor2.setControl(new VelocityDutyCycle(0.0).withSlot(0));
        throwMotor1.setControl(new VelocityDutyCycle(0.0).withSlot(0));

        if (getSensor1() || getSensor2()) {
          ampTimer.restart();
        }

        if (manualControl) {
          nextState = State.MANUAL;
        } else if (ampTimer.get() > ampDelay) {
          nextState = State.INTAKE;
        } else {
          nextState = State.AMP_SCORE;
        }
        break;

      case SPIN_UP:
        if (lastState != State.SPIN_UP) {
          indexGoalPos = indexMotor.getRotorPosition().getValueAsDouble();
          spinUpTimer.restart();
        }
        lastState = State.SPIN_UP;

        indexMotor.setControl(new MotionMagicDutyCycle(indexGoalPos).withSlot(1));
        throwMotor2.setControl(new VelocityDutyCycle(flywheelVel).withSlot(0));
        throwMotor1.setControl(new VelocityDutyCycle(flywheelVel).withSlot(0));

        if (manualControl) {
          nextState = State.MANUAL;
        } else if (!getSensor1() && !getSensor2()) {
          nextState = State.SPIN_DOWN;
        } else if (throwCommanded && (isSpunUp() || spinUpTimer.get() > spinUpDelay)) {
          nextState = State.THROW;
        } else if (ampScoreCommanded) {
          nextState = State.AMP_SCORE;
        } else {
          nextState = State.SPIN_UP;
        }
        break;

      case BACK_UP:
        if (lastState != State.BACK_UP) {
          indexGoalPos = indexMotor.getRotorPosition().getValueAsDouble() - indexOffset;
        }
        lastState = State.BACK_UP;

        indexMotor.setControl(new MotionMagicDutyCycle(indexGoalPos).withSlot(1));
        throwMotor2.setControl(new VelocityDutyCycle(0.0).withSlot(0));
        throwMotor1.setControl(new VelocityDutyCycle(0.0).withSlot(0));

        double indexPos = indexMotor.getRotorPosition().getValueAsDouble();
        if (manualControl) {
          nextState = State.MANUAL;
        } else if (!getSensor1() && !getSensor2()) {
          nextState = State.INTAKE;
        } else if (Math.abs(indexGoalPos - indexPos) < indexError) {
          nextState = State.SPIN_UP;
        } else {
          nextState = State.BACK_UP;
        }
        break;

      case INTAKE:
        lastState = State.INTAKE;

        indexMotor.setControl(new VelocityDutyCycle(intakeVel).withSlot(0));
        throwMotor2.setControl(new VelocityDutyCycle(0.0).withSlot(0));
        throwMotor1.setControl(new VelocityDutyCycle(0.0).withSlot(0));

        throwCommanded = false;
        ampScoreCommanded = false;

        if (manualControl) {
          nextState = State.MANUAL;
        } else if (getSensor2()) {
          nextState = State.BACK_UP;
        } else {
          nextState = State.INTAKE;
        }
        break;

      case MANUAL:
        if (lastState != State.INTAKE) {
          indexPowerManual = 0.0;
          flywheelPowerManual = 0.0;
        }
        lastState = State.MANUAL;

        if (!throwMotor1Failure) {
          throwMotor1.setControl(new DutyCycleOut(flywheelPowerManual));
        }
        if (!throwMotor2Failure) {
          throwMotor2.setControl(new DutyCycleOut(flywheelPowerManual));
        }
        if (!indexMotorFailure) {
          indexMotor.setControl(new DutyCycleOut(indexPowerManual));
        }

        throwCommanded = false;
        ampScoreCommanded = false;

        if (!manualControl && getSensor2()) {
          nextState = State.BACK_UP;
        } else if (!manualControl && !getSensor2()) {
          nextState = State.SPIN_DOWN;
        } else {
          nextState = State.MANUAL;
        }
        break;

      case DISABLED:
        lastState = State.DISABLED;
        nextState = State.DISABLED;
        throwCommanded = false;
        ampScoreCommanded = false;
        break;
    }
  }

  // Call when a note should be thrown. This will spin up the flywheel and release the note when the flywheel is at speed. flywheelVel is in falcon rotations per second.
  public void commandThrow(double _flywheelVel) {
    flywheelVel = _flywheelVel;
    if (!throwCommanded && (nextState == State.BACK_UP || nextState == State.SPIN_UP || nextState == State.THROW)) {
      throwCommanded = true;
    }
    ampScoreCommanded = false;
  }
  
  public void commandAmpScore() {
    flywheelVel = 0.0;
    if (!ampScoreCommanded && ((nextState == State.BACK_UP || nextState == State.SPIN_UP || nextState == State.AMP_SCORE))) {
      ampScoreCommanded = true;
    }
    throwCommanded = false;
  }

  // Call to set the flywheel velocity to a different value without throwing a note. flywheelVel is in falcon rotations per second.
  public void setFlywheelVel(double _flywheelVel) {
    if (Math.abs(flywheelVel-_flywheelVel) > allowableFlywheelVelError) {
      spinUpTimer.restart();
    }
    flywheelVel = _flywheelVel;
  }

  // Returns true if the thrower is in the process of spinning up and throwing a note.
  public boolean isThrowing() {
    return throwCommanded;
  }

  // Returns true if the thrower is in the process of scoring a note in the amp.
  public boolean isAmpScoring() {
    return ampScoreCommanded;
  }

  // Returns true if sensor 2 on the thrower is triggered.
  public boolean getSensor2() {
    return !sensor2.get();
  }

  // Returns true if sensor 1 on the thrower is triggered.
  public boolean getSensor1() {
    return !sensor1.get();
  }

  // Returns true if either the thrower motors or the index motor failed to configure on start up.
  public boolean getMotorFailure() {
    return throwMotor2Failure || indexMotorFailure || throwMotor1Failure;
  }

  // Toggles whether the thrower is under manual control.
  public void toggleManualControl() {
    manualControl = !manualControl;
  }

  // Returns true if the thrower is under manual control, and false if it automated.
  public boolean getManualControl() {
    return manualControl;
  }

  // Sets the speeds of the thrower when manualControl is true(enabled).
  public void setManualSpeeds(double _flywheelPowerManual, double _indexPowerManual) {
    flywheelPowerManual = _flywheelPowerManual;
    indexPowerManual = _indexPowerManual;
  }

  // Attempts to reboot the thrower by reconfiguring the motors. Use if trying to troubleshoot a thrower failure during a match.
  public void reboot() {
    indexMotorFailure = !configIndexMotor(indexMotor, indexMotorFailure);
    throwMotor2Failure = !configThrowMotor(throwMotor2, throwMotor2Failure);
    throwMotor1Failure = !configThrowMotor(throwMotor1, throwMotor1Failure);
    manualControl = getMotorFailure();
    init();
  }

  // Sends information about the thrower to the dashboard each period. This is handled automatically by the thrower class.
  private void updateDashboard() {
    SmartDashboard.putBoolean("manualThrowerControl", manualControl);
    SmartDashboard.putBoolean("throwerFailure", getMotorFailure());
    SmartDashboard.putBoolean("throwCommanded", throwCommanded);
    SmartDashboard.putNumber("flywheelVel", flywheelVel);
  }

  // Returns true if both flywheel motors have near 0 velocity and acceleration
  private boolean isThrowerStopped() {
    return Math.abs(throwMotor1.getRotorVelocity().getValueAsDouble()) < allowableFlywheelVelError && 
      Math.abs(throwMotor2.getRotorVelocity().getValueAsDouble()) < allowableFlywheelVelError &&
      Math.abs(throwMotor1.getAcceleration().getValueAsDouble()) < allowableFlywheelAccError && 
      Math.abs(throwMotor2.getAcceleration().getValueAsDouble()) < allowableFlywheelAccError;
  }

  // Returns true if both flywheel motors are near the desired flywheel velocity for throwing a note.
  private boolean isSpunUp() {
    return (Math.abs(throwMotor1.getRotorVelocity().getValueAsDouble() - flywheelVel) < allowableFlywheelVelError) &&
      (Math.abs(throwMotor2.getRotorVelocity().getValueAsDouble() - flywheelVel) < allowableFlywheelVelError);
  }

  // Sets PID constants, brake mode, and enforces a 40 A current limit. Returns true if the motor successfully configued.
  private boolean configThrowMotor(TalonFX _throwMotor, boolean motorFailure) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator throwMotorConfigurator = _throwMotor.getConfigurator();
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

    // Motion Magic Parameters for moving set distances
    throwMotorConfigs.Slot1.kP = 0.8;
    throwMotorConfigs.Slot1.kI = 2.0;
    throwMotorConfigs.Slot1.kD = 0.006;
    throwMotorConfigs.MotionMagic.MotionMagicAcceleration = 75.0;
    throwMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    throwMotorConfigs.MotionMagic.MotionMagicJerk = 400.0;

    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int throwMotorErrors = 0;
    while (throwMotorConfigurator.apply(throwMotorConfigs, 0.03) != StatusCode.OK) {
        throwMotorErrors++;
      motorFailure = throwMotorErrors > maxMotorFailures;
      if (motorFailure) {
        disableMotor(_throwMotor);
        return false;
      }
    }
    return true;
  }

  // Sets PID constants, brake mode, and enforces a 20 A current limit. Returns true if the motor successfully configued.
  private boolean configIndexMotor(TalonFX _indexMotor, boolean motorFailure) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator indexMotorConfigurator = _indexMotor.getConfigurator();
    TalonFXConfiguration indexMotorConfigs = new TalonFXConfiguration();

    indexMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    indexMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Inverts the direction of positive motor velocity.

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

    // Motion Magic Parameters for moving set distances
    indexMotorConfigs.Slot1.kP = 0.8;
    indexMotorConfigs.Slot1.kI = 2.0;
    indexMotorConfigs.Slot1.kD = 0.006;
    indexMotorConfigs.MotionMagic.MotionMagicAcceleration = 75.0;
    indexMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    indexMotorConfigs.MotionMagic.MotionMagicJerk = 400.0;

    // Attempts to repeatedly configure the motor up to the number of times indicated by maxMotorFailures
    int indexMotorErrors = 0;
    while (indexMotorConfigurator.apply(indexMotorConfigs, 0.03) != StatusCode.OK) {
      indexMotorErrors++;
      motorFailure = indexMotorErrors > maxMotorFailures;
      if (motorFailure) {
        disableMotor(_indexMotor);
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