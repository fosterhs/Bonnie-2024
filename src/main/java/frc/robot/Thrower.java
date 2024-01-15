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
  private final Timer intakeTimer = new Timer();
  private final Timer throwTimer = new Timer();
  private boolean previousSensor = false;
  private boolean previousFlywheelAtSpeed = false;
  private boolean throwCommanded = false;
  private double flywheelVel = 0.0;

  public Thrower() {
    configIndexMotor();
    configThrowMotor();
    intakeTimer.start();
    throwTimer.start();
  }

  // Causes the thrower to spin up and throw a note if a note is loaded.
  public void commandThrow(double _flywheelVel) {
    throwCommanded = getSensor() || throwTimer.get() < 0.3;
    flywheelVel = _flywheelVel;
  }

  // Should be called periodically during teleop and auto.
  public void periodic() {
    if (throwCommanded) {
      throwNote();
      if (throwTimer.get() > 0.6 && previousFlywheelAtSpeed) {
        throwCommanded = false;
      }
    } else {
      intakeNote();
    }
  }

  // Returns true if either proximity sensor on the thrower is triggered.
  public boolean getSensor() {
    return !sensor0.get() || !sensor1.get();
  }
  
  private void intakeNote() {
    boolean sensor = getSensor();
    if (sensor != previousSensor) {
      intakeTimer.restart();
    }
    previousSensor = sensor;

    if (getSensor() && intakeTimer.get() > 0.3) {
      throwMotor.setControl(new VelocityDutyCycle(0.0));
      indexMotor.setControl(new VelocityDutyCycle(0.0));
    } else {
      throwMotor.setControl(new VelocityDutyCycle(-2.0));
      indexMotor.setControl(new VelocityDutyCycle(-2.0));
    }
  }

  private void throwNote() {
    throwMotor.setControl(new VelocityDutyCycle(flywheelVel));

    boolean flywheelAtSpeed = Math.abs(throwMotor.getVelocity().getValueAsDouble() - flywheelVel) < 2.0;
    if (flywheelAtSpeed != previousFlywheelAtSpeed) {
        throwTimer.restart();
    }
    previousFlywheelAtSpeed = flywheelAtSpeed;

    if (flywheelAtSpeed) {
      indexMotor.setControl(new VelocityDutyCycle(flywheelVel));
    }
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