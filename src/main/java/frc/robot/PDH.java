package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PDH {
  PowerDistribution pdh;
    
  public PDH() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
    pdh.setSwitchableChannel(true);
  }

  public void updateDash() {
    SmartDashboard.putNumber("PDH Total Current (A)", pdh.getTotalCurrent());
    SmartDashboard.putNumber("PDH Voltage (V)", pdh.getVoltage());
    SmartDashboard.putBoolean("PDH Switchable Channel", pdh.getSwitchableChannel());
  }
}