package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PDH {
  PowerDistribution pdh;
    
  public PDH() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
    pdh.setSwitchableChannel(true); // Enables channel 23 on the power distribution hub.
  }

  // Publishes supply voltage and current information to the dashboard. Also publishes whether the switchable channel is active. 
  public void updateDash() {
    SmartDashboard.putNumber("PDH Total Current (A)", pdh.getTotalCurrent());
    SmartDashboard.putNumber("PDH Voltage (V)", pdh.getVoltage());
    SmartDashboard.putBoolean("PDH Switchable Channel", pdh.getSwitchableChannel());
  }
}