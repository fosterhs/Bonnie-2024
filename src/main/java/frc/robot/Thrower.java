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
  private final TalonFX throwMotor = new TalonFX(10);
  private final TalonFX indexMotor = new TalonFX(9);
  private boolean throwMotorFailure = false;
  private boolean indexMotorFailure = false;
  private final DigitalInput sensor0 = new DigitalInput(0);
  private final DigitalInput sensor1 = new DigitalInput(1);
  private final Timer noteLoadedTimer = new Timer();
  private final Timer noteUnloadedTimer = new Timer();
  private final Timer spinUpTimer = new Timer();
  private boolean pastSensor = true;
  private boolean currSensor = true;
  private boolean pastIsSpinningUp = false;
  private boolean currIsSpinningUp = false;
  private boolean isSpunUp = false;
  private boolean throwCommanded = false;
  private double flywheelVel = 80.0;

  public Thrower() {
    configIndexMotor();
    configThrowMotor();
    noteLoadedTimer.start();
    noteUnloadedTimer.start();
    spinUpTimer.start();
  }

  // Should be called when a note would like to be thrown.
  public void commandThrow(double _flywheelVel) {
    flywheelVel = _flywheelVel;
    throwCommanded = true;
  }

  // Should be called periodically during teleop and auto.
  public void periodic() {
    currSensor = getSensor();
    if (currSensor && !pastSensor) {
      noteLoadedTimer.restart();
    } else if (!currSensor && pastSensor) {
      noteUnloadedTimer.restart();
    }
    pastSensor = currSensor;

    isSpunUp = Math.abs(throwMotor.getVelocity().getValueAsDouble() - flywheelVel) < 2.0 || spinUpTimer.get() > 1.0;
    currIsSpinningUp = !isSpunUp && currSensor && noteLoadedTimer.get() > 0.3;
    if (currIsSpinningUp && !pastIsSpinningUp) {
      spinUpTimer.restart();
    }
    pastIsSpinningUp = currIsSpinningUp;

    throwCommanded = throwCommanded && (getSensor() || noteUnloadedTimer.get() < 0.6);

    if (throwCommanded && isSpunUp) {
      throwNote();
    } else if (!throwCommanded && (!currSensor || noteLoadedTimer.get() < 0.3)) {
      intakeNote();
    } else {
      spinUp();
    }
  }

  public boolean isThrowing() {
    return throwCommanded;
  }

  // Returns true if either proximity sensor on the thrower is triggered.
  // Also restarts the relevant timer if a change in the sensor reading is detected.
  public boolean getSensor() {
    return !sensor0.get() || !sensor1.get();
  }
  
  private void throwNote() {
    throwMotor.setControl(new VelocityDutyCycle(flywheelVel));
    indexMotor.setControl(new VelocityDutyCycle(flywheelVel));
  }

  private void spinUp() {
    throwMotor.setControl(new VelocityDutyCycle(flywheelVel));
    indexMotor.setControl(new VelocityDutyCycle(0.0));
  }

  private void intakeNote() {
    throwMotor.setControl(new VelocityDutyCycle(-2.0));
    indexMotor.setControl(new VelocityDutyCycle(-2.0));
  }

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