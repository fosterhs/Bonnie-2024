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
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Thrower {
  private final CANSparkFlex vortex1 = new CANSparkFlex(1, MotorType.kBrushless);
  private final CANSparkFlex vortex2 = new CANSparkFlex(2, MotorType.kBrushless);
  private final TalonFX indexMotor = new TalonFX(13, "rio"); // The motor running the intake.
  // Initializes the proximity sensors. These return false if an object is detected and true if no object is detected.
  private final DigitalInput sensor1 = new DigitalInput(3); // Sensor closest to the intake. Notes will trigger this sensor first when intaked normally.
  private final DigitalInput sensor2 = new DigitalInput(4); // Sensor in the middle. Notes will trigger this sensor second when intaked normally.
  private final DigitalInput sensor3 = new DigitalInput(5); // Sensor closest to the shooter. Notes will trigger this sensor third when intaked normally.
  private final double motorCurrentLimit = 40.0; // Index motor current limit in amps. Should be based on the breaker used in the PDP.
  private final int maxMotorFailures = 3; // The number of times a motor will attempt to reconfigure before declaring a failure and putting the thrower into a manual state.
  private final int maxVortexFailures = 50;
  private boolean vortexFailure = false;
  private final double intakeVel = 50.0; // The number of rotations per second that the motors will spin in reverse when intaking a note.
  private final double ampVel = 75.0; // The number of rotations per second that the motors will spin forwards when scoing a note in the amp.
  private final double scoreVel = 120.0; // The number of rotations per second that the motors will spin forwards when loading a note into the flywheels
  private final double backUpVel = 20.0; // The number of rotations per second that the motors will spin backwards when reversing a note in the BACK_UP state.
  private final double allowableFlywheelVelError = 2.0; // The number of rotations per second of error in the flywheel velocity that is acceptable before a note begins to be launched.
  private final double spinUpDelay = 4.0; // The amount of time in seconds that the thrower motor is allowed to stay at 100% power without attaining the commanded flywheel velocity before the note is thrown. This value should correspond to the amount of time the thrower motor takes to spin up to full speed.
  private final double throwDelay = 1.0; // The amount of time the flywheel will keep spinning after the note is no longer detected. Ensures the note has exited the flywheel before spinning down.
  private final double ampDelay = 1.5; // The amount of time the thrower will keep backfeeding after the note is no longer detected. Ensures the note has exited before spinning down.
  private final Timer throwTimer = new Timer(); // Keeps track of how long it has been since the note was last detected in the AMP_SCORE state.
  private final Timer ampTimer = new Timer(); // Keeps track of how long it has been since the note was last detected in the THROW state.
  private final Timer spinUpTimer = new Timer(); // Keeps track of how long the thrower has been in the SPIN_UP state.
  private double vortex1Vel = 4000.0; // The top one.
  private double vortex2Vel = 4000.0; // The bottom one.

  // Keeps track of the different states of the thrower.
  private enum ThrowerState {
    THROW,
    SPIN_UP,
    BACK_UP,
    INTAKE,
    MANUAL,
    DISABLED,
    AMP_SCORE;
  } 
  private ThrowerState nextState;
  private ThrowerState lastState;

  // Indicates whether the motor failed to configure on startup. Each motor will attempt to configure up to the number of times specified by maxMotorFailures
  private boolean indexMotorFailure = false;

  private boolean ampScoreCommanded = false; // Returns true if an amp score command was recieved, but not yet executed.
  private boolean throwCommanded = false; // Returns true if a throw command was recieved, but not yet executed. Reverts to false if a note is not detected.
  private boolean disableFlywheel = false; // The flywheel will not spin up if this is true. Used to conserve power.

  // These variables store the desired motor velocities which are used and updated when the thrower is in the MANUAL state.
  private boolean manualControl = false;
  private double flywheelPowerManual = 0.0;
  private double indexPowerManual = 0.0;

  private double indexMotorGoalPos = 0.0; // Stores the goal position of the index motor. Used in the BACK_UP state.
  private final double indexMotorOffset = 1.0; // How far the note should be backed off in the BACK_UP state in falcon rotations.

  public Thrower() {
    reboot();
  }

  // Should be called once teleopInit() and autoInit() sections of the main robot code. Neccesary for the class to function.
  public void init() {
    lastState = ThrowerState.DISABLED;
    if (manualControl) {
      nextState = ThrowerState.MANUAL;
    } else if (getSensor1() || getSensor2() || getSensor3()) {
      nextState = ThrowerState.SPIN_UP;
    } else {
      nextState = ThrowerState.INTAKE;
    }
    throwTimer.restart();
    spinUpTimer.restart();
    ampTimer.restart();
    throwCommanded = false;
    ampScoreCommanded = false;
  }

  // Should be called once teleopPeriodic() and autoPeriodic() sections of the main robot code. Neccesary for the class to function.
  public void periodic() {
    switch (nextState) {
      case THROW:
        if (lastState != ThrowerState.THROW) {
          throwTimer.restart();
        }
        lastState = ThrowerState.THROW;

        vortex1.getPIDController().setReference(vortex1Vel, ControlType.kSmartVelocity, 0);
        vortex2.getPIDController().setReference(vortex2Vel, ControlType.kSmartVelocity, 0);
        indexMotor.setControl(new VelocityDutyCycle(scoreVel).withSlot(0).withEnableFOC(true));

        if (getSensor1() || getSensor2() || getSensor3()) {
          throwTimer.restart();
        }
        ampScoreCommanded = false;

        if (manualControl) {
          nextState = ThrowerState.MANUAL;
        } else if (throwTimer.get() > throwDelay) {
          nextState = ThrowerState.INTAKE;
        } else {
          nextState = ThrowerState.THROW;
        }
        break;

      case AMP_SCORE:
        if (lastState != ThrowerState.AMP_SCORE) {
          ampTimer.restart();
        }
        lastState = ThrowerState.AMP_SCORE;

        indexMotor.setControl(new VelocityDutyCycle(-ampVel).withSlot(0).withEnableFOC(true));
        vortex1.set(0.0);
        vortex2.set(0.0);

        if (getSensor1() || getSensor2() || getSensor3()) {
          ampTimer.restart();
        }
        throwCommanded = false;

        if (manualControl) {
          nextState = ThrowerState.MANUAL;
        } else if (ampTimer.get() > ampDelay) {
          nextState = ThrowerState.INTAKE;
        } else {
          nextState = ThrowerState.AMP_SCORE;
        }
        break;

      case SPIN_UP:
        if (lastState != ThrowerState.SPIN_UP) {
          spinUpTimer.restart();
          indexMotorGoalPos = indexMotor.getRotorPosition().getValueAsDouble() - indexMotorOffset;
        }
        lastState = ThrowerState.SPIN_UP;

        indexMotor.setControl(new MotionMagicDutyCycle(indexMotorGoalPos).withSlot(1).withEnableFOC(true));
        if (disableFlywheel) {
          vortex1.set(0.0);
          vortex2.set(0.0);
        } else {
          vortex1.getPIDController().setReference(vortex1Vel, ControlType.kSmartVelocity, 0);
          vortex2.getPIDController().setReference(vortex2Vel, ControlType.kSmartVelocity, 0);
        }

        if (manualControl) {
          nextState = ThrowerState.MANUAL;
        } else if (!getSensor1() && !getSensor2() && !getSensor3()) {
          nextState = ThrowerState.INTAKE;
        } else if (throwCommanded && (spinUpTimer.get() > spinUpDelay || isSpunUp())) {
          nextState = ThrowerState.THROW;
        } else if (ampScoreCommanded) {
          nextState = ThrowerState.AMP_SCORE;
        } else {
          nextState = ThrowerState.SPIN_UP;
        }
        break;

      case BACK_UP:
        lastState = ThrowerState.BACK_UP;

        indexMotor.setControl(new VelocityDutyCycle(-backUpVel).withSlot(0).withEnableFOC(true));
        vortex1.set(0.0);
        vortex2.set(0.0);

        if (manualControl) {
          nextState = ThrowerState.MANUAL;
        } else if (!getSensor1() && !getSensor2() && !getSensor3()) {
          nextState = ThrowerState.INTAKE;
        } else if (!getSensor3()) {
          nextState = ThrowerState.SPIN_UP;
        } else {
          nextState = ThrowerState.BACK_UP;
        }
        break;

      case INTAKE:
        lastState = ThrowerState.INTAKE;

        indexMotor.setControl(new VelocityDutyCycle(intakeVel).withSlot(0).withEnableFOC(true));
        vortex1.set(0.0);
        vortex2.set(0.0);
        
        // Prevents integer overflow issues.
        if (indexMotor.getRotorPosition().getValueAsDouble() > 1000.0) {
          indexMotor.setPosition(0.0);
        }

        throwCommanded = false;
        ampScoreCommanded = false;

        if (manualControl) {
          nextState = ThrowerState.MANUAL;
        } else if (getSensor3()) {
          nextState = ThrowerState.BACK_UP;
        } else {
          nextState = ThrowerState.INTAKE;
        }
        break;

      case MANUAL:
        if (lastState != ThrowerState.MANUAL) {
          indexPowerManual = 0.0;
          flywheelPowerManual = 0.0;
        }
        lastState = ThrowerState.MANUAL;
        if (!vortexFailure) {
          vortex1.set(flywheelPowerManual);
          vortex2.set(flywheelPowerManual);
        }
        if (!indexMotorFailure) {
          indexMotor.setControl(new DutyCycleOut(indexPowerManual).withEnableFOC(true));
        }

        throwCommanded = false;
        ampScoreCommanded = false;

        if (!manualControl && getSensor3()) {
          nextState = ThrowerState.BACK_UP;
        } else if (!manualControl && !getSensor3()) {
          nextState = ThrowerState.INTAKE;
        } else {
          nextState = ThrowerState.MANUAL;
        }
        break;

      case DISABLED:
        lastState = ThrowerState.DISABLED;
        nextState = ThrowerState.DISABLED;
        throwCommanded = false;
        ampScoreCommanded = false;
        break;
    }
  }

  // Call when a note should be thrown. This will spin up the flywheel and release the note when the flywheel is at speed. flywheelVel is in falcon rotations per second.
  public void commandThrow() {
    setDisableFlywheel(false);
    if (!throwCommanded && (nextState == ThrowerState.BACK_UP || nextState == ThrowerState.SPIN_UP || nextState == ThrowerState.THROW)) {
      throwCommanded = true;
    }
    ampScoreCommanded = false;
  }
  
  public void commandAmpScore() {
    setDisableFlywheel(true);
    if (!ampScoreCommanded && ((nextState == ThrowerState.BACK_UP || nextState == ThrowerState.SPIN_UP || nextState == ThrowerState.AMP_SCORE))) {
      ampScoreCommanded = true;
    }
    throwCommanded = false;
  }

  public void setDisableFlywheel(boolean _disableFlywheel) {
    if (!_disableFlywheel && disableFlywheel) {
      spinUpTimer.restart();
    }
    disableFlywheel = _disableFlywheel;
  }

  // Call to set the flywheel velocity to a different value without throwing a note. flywheelVel is in falcon rotations per second.
  public void setFlywheelVel(double _vortex1Vel, double _vortex2Vel) {
    if (Math.abs(vortex1Vel - _vortex1Vel) > allowableFlywheelVelError || Math.abs(vortex2Vel - _vortex2Vel) > allowableFlywheelVelError) {
      spinUpTimer.restart();
    }
    vortex1Vel = _vortex1Vel;
    vortex2Vel = _vortex2Vel;
  }

  // Returns true if the thrower is in the process of spinning up and throwing a note.
  public boolean isThrowing() {
    return throwCommanded;
  }

  // Returns true if the thrower is in the process of scoring a note in the amp.
  public boolean isAmpScoring() {
    return ampScoreCommanded;
  }

  public boolean isSpunUp() {
    return Math.abs(vortex1.getEncoder().getVelocity() - vortex1Vel) < allowableFlywheelVelError && Math.abs(vortex2.getEncoder().getVelocity() - vortex2Vel) < allowableFlywheelVelError;
  }

  // Returns true if sensor 1 on the thrower is triggered.
  public boolean getSensor3() {
    return !sensor3.get();
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
    return indexMotorFailure;
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
    manualControl = getMotorFailure();
    vortexFailure = false;
    configVortex();
    init();
  }

  // Sends information about the thrower to the dashboard each period. This is handled automatically by the thrower class.
  public void updateDashboard() {
    SmartDashboard.putBoolean("manualThrowerControl", manualControl);
    SmartDashboard.putBoolean("throwerFailure", getMotorFailure());
    SmartDashboard.putBoolean("throwCommanded", throwCommanded);
    SmartDashboard.putBoolean("ampScoreCommanded", ampScoreCommanded);
    SmartDashboard.putBoolean("Sensor 1", getSensor1());
    SmartDashboard.putBoolean("Sensor 2", getSensor2());
    SmartDashboard.putBoolean("Sensor 3", getSensor3());
    SmartDashboard.putNumber("Vortex 1 Vel", vortex1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Vortex 2 Vel", vortex2.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Vortex Failure", vortexFailure);
  }

  private void configVortex() {
    int motorErrors = 0;
    while (vortex1.restoreFactoryDefaults() != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setP(0.00043, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setI(0.000004, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setD(0.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setFF(0.000163, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setSmartMotionMaxAccel(8000.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setSmartMotionMaxVelocity(6600.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.getPIDController().setIMaxAccum(0.05, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.setSmartCurrentLimit(80) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex1.burnFlash() != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    vortex1.setInverted(true);

    while (vortex2.restoreFactoryDefaults() != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setP(0.00043, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setI(0.000004, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setD(0.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setFF(0.000163, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setSmartMotionMaxAccel(8000.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setSmartMotionMaxVelocity(6600.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.getPIDController().setIMaxAccum(0.05, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.setSmartCurrentLimit(80) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
    while (vortex2.burnFlash() != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxVortexFailures) {
        vortexFailure = true;
        break;
      }
    }
  }

  // Sets PID constants, brake mode, and enforces a 20 A current limit. Returns true if the motor successfully configued.
  private boolean configIndexMotor(TalonFX _indexMotor, boolean motorFailure) {
    // Creates a configurator and config object to configure the motor.
    TalonFXConfigurator indexMotorConfigurator = _indexMotor.getConfigurator();
    TalonFXConfiguration indexMotorConfigs = new TalonFXConfiguration();

    indexMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Motor brakes instead of coasting.
    indexMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverts the direction of positive motor velocity.

    // Setting current limits
    indexMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMotorConfigs.CurrentLimits.SupplyCurrentLimit = motorCurrentLimit;
    indexMotorConfigs.CurrentLimits.SupplyCurrentThreshold = motorCurrentLimit;
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