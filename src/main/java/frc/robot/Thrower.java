package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
  private final CANSparkFlex vortex1 = new CANSparkFlex(1, MotorType.kBrushless); // The top flywheel motor.
  private final CANSparkFlex vortex2 = new CANSparkFlex(2, MotorType.kBrushless); // The bottom flywheel motor.
  private final TalonFX indexMotor = new TalonFX(13, "rio"); // The motor running the intake rollers.
  private boolean indexMotorFailure = false; // Indicates whether the motor failed to configure on startup.
  private boolean vortex1Failure = false; // Indicates whether the motor failed to configure on startup.
  private boolean vortex2Failure = false; // Indicates whether the motor failed to configure on startup.

  // Initializes the proximity sensors. These return false if an object is detected and true if no object is detected.
  private final DigitalInput sensor1 = new DigitalInput(3); // Sensor closest to the intake. Notes will trigger this sensor first when intaked normally.
  private final DigitalInput sensor2 = new DigitalInput(4); // Sensor in the middle. Notes will trigger this sensor second when intaked normally.
  private final DigitalInput sensor3 = new DigitalInput(5); // Sensor closest to the shooter. Notes will trigger this sensor third when intaked normally.

  // Keeps track of the different states of the thrower.
  private enum ThrowerState {
    THROW,
    SPIN_UP,
    BACK_UP,
    INTAKE,
    DISABLED,
    AMP_SCORE;
  } 
  private ThrowerState nextState;
  private ThrowerState lastState;

  private final double intakeVel = 50.0; // The number of rotations per second that the motors will spin in reverse when intaking a note.
  private final double ampVel = 75.0; // The number of rotations per second that the motors will spin forwards when scoing a note in the amp.
  private final double scoreVel = 120.0; // The number of rotations per second that the motors will spin forwards when loading a note into the flywheels
  private final double backUpVel = 20.0; // The number of rotations per second that the motors will spin backwards when reversing a note in the BACK_UP state.
  private final double allowableFlywheelVelError = 2.0; // The number of rotations per second of error in the flywheel velocity that is acceptable before a note begins to be launched.
  private final double spinUpDelay = 1.2; // The amount of time in seconds that the thrower motor is allowed to stay at 100% power without attaining the commanded flywheel velocity before the note is thrown. This value should correspond to the amount of time the thrower motor takes to spin up to full speed.
  private final double throwDelay = 0.25; // The amount of time the flywheel will keep spinning after the note is no longer detected. Ensures the note has exited the flywheel before spinning down.
  private final double ampDelay = 1.5; // The amount of time the thrower will keep backfeeding after the note is no longer detected. Ensures the note has exited before spinning down.
  private final double indexMotorOffset = 1.0; // How far the note should be backed off in the BACK_UP state in falcon rotations.

  private final Timer throwTimer = new Timer(); // Keeps track of how long it has been since the note was last detected in the AMP_SCORE state.
  private final Timer ampTimer = new Timer(); // Keeps track of how long it has been since the note was last detected in the THROW state.
  private final Timer spinUpTimer = new Timer(); // Keeps track of how long the thrower has been in the SPIN_UP state.

  private double vortex1FlywheelVel = 4000.0; // The desired speed of the top flywheel in rotations per minute as a note is being thrown.
  private double vortex2FlywheelVel = 4000.0; // The desired speed of the bottom flywheel in rotations per minute as a note is being thrown.
  private boolean ampScoreCommanded = false; // Returns true if an amp score command was recieved, but not yet executed.
  private boolean throwCommanded = false; // Returns true if a throw command was recieved, but not yet executed. Reverts to false if a note is not detected.
  private boolean disableFlywheel = false; // The flywheel will not spin up if this is true. Used to conserve power.
  private double indexMotorGoalPos = 0.0; // Stores the goal position of the index motor. Used in the BACK_UP state.

  public Thrower() {
    indexMotorFailure = !configIndexMotor(indexMotor, false, 60.0, 3);
    vortex1Failure = !configVortex(vortex1, true, 80, 3);
    vortex2Failure = !configVortex(vortex2, false, 80, 3);
  }

  // Should be called once teleopInit() and autoInit() sections of the main robot code. Neccesary for the class to function.
  public void init() {
    lastState = ThrowerState.DISABLED;
    if (getSensor1() || getSensor2() || getSensor3()) {
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

        vortex1.getPIDController().setReference(vortex1FlywheelVel, ControlType.kSmartVelocity, 0);
        vortex2.getPIDController().setReference(vortex2FlywheelVel, ControlType.kSmartVelocity, 0);
        indexMotor.setControl(new VelocityDutyCycle(scoreVel).withSlot(0).withEnableFOC(true));

        if (getSensor1() || getSensor2() || getSensor3()) {
          throwTimer.restart();
        }
        ampScoreCommanded = false;

        if (throwTimer.get() > throwDelay) {
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

        if (ampTimer.get() > ampDelay) {
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
          vortex1.getPIDController().setReference(vortex1FlywheelVel, ControlType.kSmartVelocity, 0);
          vortex2.getPIDController().setReference(vortex2FlywheelVel, ControlType.kSmartVelocity, 0);
        }

        if (!getSensor1() && !getSensor2() && !getSensor3()) {
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

        if (!getSensor1() && !getSensor2() && !getSensor3()) {
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

        if (getSensor3()) {
          nextState = ThrowerState.BACK_UP;
        } else {
          nextState = ThrowerState.INTAKE;
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
  
  // Call when the robot is ready to score a note into the amp. This will discharge the note backwards through the intake. 
  public void commandAmpScore() {
    setDisableFlywheel(true);
    if (!ampScoreCommanded && ((nextState == ThrowerState.BACK_UP || nextState == ThrowerState.SPIN_UP || nextState == ThrowerState.AMP_SCORE))) {
      ampScoreCommanded = true;
    }
    throwCommanded = false;
  }

  // Disables the flywheel if true is passed in. Useful for amp scoring, where the flywheel is not neccessary.
  public void setDisableFlywheel(boolean _disableFlywheel) {
    if (!_disableFlywheel && disableFlywheel) {
      spinUpTimer.restart();
    }
    disableFlywheel = _disableFlywheel;
  }

  // Call to set the flywheel velocity to a different value without throwing a note. flywheelVel is in rotations per minute.
  public void setFlywheelVel(double _vortex1Vel, double _vortex2Vel) {
    if (Math.abs(vortex1FlywheelVel - _vortex1Vel) > allowableFlywheelVelError || Math.abs(vortex2FlywheelVel - _vortex2Vel) > allowableFlywheelVelError) {
      spinUpTimer.restart();
    }
    vortex1FlywheelVel = _vortex1Vel;
    vortex2FlywheelVel = _vortex2Vel;
  }

  // Returns true if the thrower is in the process of spinning up and throwing a note.
  public boolean isThrowing() {
    return throwCommanded;
  }

  // Returns true if the thrower is in the process of scoring a note in the amp.
  public boolean isAmpScoring() {
    return ampScoreCommanded;
  }

  // Returns true if both flywheel motors are at the desired velocity, within the specified tolerance.
  public boolean isSpunUp() {
    return Math.abs(vortex1.getEncoder().getVelocity() - vortex1FlywheelVel) < allowableFlywheelVelError && Math.abs(vortex2.getEncoder().getVelocity() - vortex2FlywheelVel) < allowableFlywheelVelError;
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

  // Returns true if the index motor failed to configure on start up.
  public boolean getIndexMotorFailure() {
    return indexMotorFailure;
  }

  // Returns true if the flywheel vortex motor failed to configure on start up.
  public boolean getVortex1Failure() {
    return vortex1Failure;
  }

  // Returns true if the flywheel vortex motor failed to configure on start up.
  public boolean getVortex2Failure() {
    return vortex2Failure;
  }

  // Sends information about the thrower to the dashboard each period. This is handled automatically by the thrower class.
  public void updateDashboard() {
    SmartDashboard.putBoolean("Thrower isSpunUp", isSpunUp());
    SmartDashboard.putBoolean("Thrower throwCommanded", isThrowing());
    SmartDashboard.putBoolean("Thrower ampScoreCommanded", isAmpScoring());
    SmartDashboard.putBoolean("Thrower Sensor 1", getSensor1());
    SmartDashboard.putBoolean("Thrower Sensor 2", getSensor2());
    SmartDashboard.putBoolean("Thrower Sensor 3", getSensor3());
    SmartDashboard.putNumber("Thrower Vortex 1 Velocity", vortex1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Thrower Vortex 2 Velocity", vortex2.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Thrower Index Motor Failure", getIndexMotorFailure());
    SmartDashboard.putBoolean("Thrower Vortex 1 Motor Failure", getVortex1Failure());
    SmartDashboard.putBoolean("Thrower Vortex 2 Motor Failure", getVortex2Failure());
  }

  // Attempts to configure the flywheel motors. Sets inverts, neutral mode, PID constants, and current limit. Returns true if the motor successfully configued.
  private boolean configVortex(CANSparkFlex motor, boolean invert, int currentLimit, int maxMotorErrors) {
    int motorErrors = 0;
    while (motor.restoreFactoryDefaults() != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setP(0.00043, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setI(0.000004, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setD(0.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setFF(0.000163, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setSmartMotionMaxAccel(8000.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setSmartMotionMaxVelocity(6600.0, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.getPIDController().setIMaxAccum(0.05, 0) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    while (motor.setSmartCurrentLimit(currentLimit) != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    motor.setInverted(invert);
    while (motor.burnFlash() != REVLibError.kOk) {
      motorErrors++;
      if (motorErrors > maxMotorErrors) {
        return false;
      }
    }
    Timer.delay(0.8);
    return true;
  }

  // Attempts to configure the index motor. Sets inverts, neutral mode, and PID constants. Returns true if the motor successfully configued.
  private boolean configIndexMotor(TalonFX motor, boolean invert, double currentLimit, int maxMotorErrors) {
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
  
    // Setting Velocity PID parameters
    motorConfigs.Slot0.kP = 0.008;
    motorConfigs.Slot0.kI = 0.06;
    motorConfigs.Slot0.kD = 0.0002;
    motorConfigs.Slot0.kV = 0.009;

    // Setting Motion Magic parameters
    motorConfigs.Slot1.kP = 0.8;
    motorConfigs.Slot1.kI = 2.0;
    motorConfigs.Slot1.kD = 0.006;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 75.0;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 50.0;
    motorConfigs.MotionMagic.MotionMagicJerk = 400.0;

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