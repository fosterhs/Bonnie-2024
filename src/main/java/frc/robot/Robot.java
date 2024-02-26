package frc.robot;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.
  private final XboxController operator = new XboxController(1); // Initializes the operator controller.

  // Limits the acceleration of controller inputs. 
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop/Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop/Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAccTeleop/Drivetrain.maxAngularVelTeleop);
  
  private double speedScaleFactor = 1.0; // Scales the speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Thrower thrower = new Thrower();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();

  // Auto Chooser Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "Auto 1";
  private static final String auto2 = "Auto 2";
  private static final String auto3 = "Auto 3"; 
  private static final String auto4 = "Auto 4"; 
  private String autoSelected;
  private int autoStage = 1;

  private boolean lastIsAmpScoring = false; // Stores whether the thrower was amp scoring in the previous period.
  private final Timer ampTimer = new Timer(); // Controls the inclination of the arm during amp scoring.

  private final CANdle candle = new CANdle(0); // Initialzes the LEDs

  // Arm States (Teleop)
  private enum ArmState {
    DRIVE,
    SHOOT,
    AMP,
    INTAKE,
    MANUAL_SHOOT;
  } 
  ArmState currArmState = ArmState.DRIVE; // Stores the current arm state. The robot will default to the value intialized here when teleop is first entered.
  private final double armDriveSetpoint = 90.0; // The arm's driving position in degrees.
  private final double armAmpSetpoint = 52.0; // The arm's inital amp scoring position in degrees.
  private final double armIntakeSetpoint = -3.0; // The arm's intake position in degrees.
  private final double armAmpRaiseRate = 6.0; // The rate at which the arm is raised during amp scoring in deg/sec.
  private final double armManualSetpoint = 8.0; // THe arm's manual shooting position in degrees.
  private final Timer armTimer = new Timer(); // Tracks the number of secound that the arm is at the setpoint 

  public void robotInit() {
    // Allows the user to choose which auto to do
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    SmartDashboard.putData("Autos", autoChooser);

    ampTimer.restart(); // Gets the amp timer started. Used in teleop to incline the arm.
    createToggles(); // Creates the infrastructure for using dashboard toggles.
    armTimer.restart(); // Gets the arm timer started.

    swerve.loadPath("Test", 0.0, 0.0, 0.0, 180.0); // Loads the path. All paths should be loaded in robotInit() because this call is computationally expensive.

    // Helps prevent loop overruns when the robot is first enabled. These calls cause the robot to initialize code in other parts of the program so it does not need to be initialized during autonomousInit() or teleopInit(), saving computational resources.
    swerve.resetDriveController(0.0);
    swerve.aimDrive(0.1, 0.0, 0.0, true);
    swerve.driveTo(0.0, 0.0, 0.0);
    swerve.addCalibrationEstimate();
    swerve.pushCalibration();
    swerve.resetCalibration();
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.atPathEndpoint(0);
    swerve.drive(0.1, 0.0, 0.0, false, 0.0, 0.0);
    swerve.resetOdometry(0, 0, 0);
    swerve.updateDash();
    climber.set(0.0, 0.0);
    climber.periodic();
    arm.atSetpoint();
    arm.periodic();
    arm.updateSetpoint(armDriveSetpoint);
    arm.setManualPower(0.0);
    thrower.init();
    thrower.periodic();
  }

  public void robotPeriodic() {
    updateVision(); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    swerve.updateDash(); // Pushes drivetrain information to the Dashboard.
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    updateToggles(); // Checks the dashboard toggles and takes any actions based on them.

    // Sets the LEDs on if the arm is at the setpoint.
    if (arm.atSetpoint()) {
      candle.setLEDs(0, 255, 0, 0, 0, 8);
    } else {
      candle.setLEDs(255, 0, 255, 0, 0, 8);
    }

    if (!arm.atSetpoint()) { // Resets the arm timer to 0 if the arm is not at the current setpoint.
      armTimer.restart();
    }

    // Re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
    if (driver.getRawButtonPressed(8)) {
      swerve.resetGyro();
    }
  }
  
  public void autonomousInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    thrower.init(); // Must be called during autoInit() and teleopInit() for the thrower to work properly.
    arm.init();
    armTimer.restart();
    autoStage = swerve.isCalibrated() ? 1 : -1; // Goes to default case if April Tags were not visible. 
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setFlywheelVel(120.0);
        break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setFlywheelVel(120.0);
        break;

      case auto3: 
        // AutoInit 3 code goes here.
        break;

      case auto4:
        // AutoInit 4 code goes here.
        break;
    }
  }

  public void autonomousPeriodic() {
    thrower.periodic();
    arm.periodic();
    climber.periodic();
    SmartDashboard.putNumber("autoStage", autoStage);
    SmartDashboard.putNumber("ArmTimer", armTimer.get());
    SmartDashboard.putBoolean("atGoal", swerve.atDriveGoal());
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1: 
            // Auto 1 code goes here.
            swerve.aimDrive(0.0, 0.0, getAimHeading(), true);
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once (like an autoStageInit()), then move on to the next stage.
                armTimer.restart();
                arm.updateSetpoint(armDriveSetpoint);
                autoStage = -1; // Goes to default case.
              }
            }
            break;

          default: 
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot after auto is completed.
            break;
        }
        break;

      case auto2:
        // Auto 2 code goes here.
        switch (autoStage) {
          case 1:
            swerve.aimDrive(0.0, 0.0, getAimHeading(), true);
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once (like an autoStageInit()), then move on to the next stage.
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 2;
              }
            }
            break;

          case 2:
            swerve.aimDrive(0.0, 0.0, 180.0, true);

            if (armTimer.get() > 0.3) {  
             autoStage = 3;
            }
            break;

          case 3:
            swerve.aimDrive(1.0, 0.0, 180.0, true);

            if (thrower.getSensor1()) {
              swerve.resetDriveController(getAimArmAngle());
              arm.updateSetpoint(getAimArmAngle());
              thrower.setFlywheelVel(120.0);
              armTimer.restart();
              autoStage = 4;
            } else if (swerve.getXPos() > 4.0) {
              autoStage = -1; // Goes to default case. 
            }
            break;

          case 4:
            swerve.aimDrive(0.0, 0.0, getAimHeading(), true);
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once (like an autoStageInit()), then move on to the next stage.
                autoStage = -1; // Default case
              }
            }
            break;

          default: 
            swerve.drive(0.0, 0.0, 0.0, false, 0, 0);
            break;
        }
        break;

      case auto3: 
        // Auto 3 code goes here.
        break;

      case auto4: 
        // Auto 4 code goes here.
        break;

      default:
        swerve.drive(0.0, 0.0, 0.0, false, 0, 0);
        break;
    }
  }

  public void teleopInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    thrower.init(); // Must be called during autoInit() and teleopInit() for the thrower to work properly.
    arm.init();
  }

  public void teleopPeriodic() {
    if (driver.getRawButtonPressed(4)) { // Y Button
      speedScaleFactor = 0.15;
    }
    if (driver.getRawButtonPressed(2)) { // B button
      speedScaleFactor = 0.6;
    }
    if (driver.getRawButtonPressed(1)) { // A button
      speedScaleFactor = 1.0;
    }

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*speedScaleFactor)*Drivetrain.maxAngularVelTeleop;

    // Auto Rotate to Aim Heading
    if (driver.getRawButtonPressed(6)) { // Right Bumper
      swerve.resetDriveController(getAimHeading());
    }
    if (driver.getRawButton(6)) { // Right Bumper
      swerve.aimDrive(xVel, yVel, getAimHeading(), true); // Rotate to speaker.
    } else if (driver.getRightTriggerAxis() > 0.25) { // Right Trigger
      double ampHeading = swerve.isBlueAlliance() ? -90.0 : 90.0;
      swerve.aimDrive(xVel, yVel, ampHeading, true); // Rotate to amp.
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary.
    if (driver.getRawButtonPressed(7)) {
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) {
      swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButtonReleased(7)) {
      swerve.pushCalibration(); // Updates the position of the robot on the field based on previous calculations.
    }

    arm.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the arm.
    if (arm.getManualControl()) {
      arm.setManualPower(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
    } else {
      if (operator.getRawButtonPressed(1)) { // A Bytton
        currArmState = ArmState.DRIVE;
      }
      if (operator.getRawButtonPressed(2)) { // B Button
        currArmState = ArmState.INTAKE;
      }
      if (operator.getRawButtonPressed(3)) { // X Button
        currArmState = ArmState.SHOOT;
      }
      if (operator.getRawButtonPressed(4)) { // Y Button
        currArmState = ArmState.AMP;
      }
      if (operator.getRawButtonPressed(8)) { // Menu Button
        currArmState = ArmState.MANUAL_SHOOT;
      }
      switch (currArmState) {
        case INTAKE:
          arm.updateSetpoint(armIntakeSetpoint);
          thrower.setFlywheelVel(0.0);
          lastIsAmpScoring = false;
          break;

        case DRIVE:
          arm.updateSetpoint(armDriveSetpoint);
          thrower.setFlywheelVel(0.0);
          lastIsAmpScoring = false;
          break;

        case SHOOT:
          arm.updateSetpoint(getAimArmAngle()); 
          thrower.setFlywheelVel(120.0);
          lastIsAmpScoring = false;
          break;

        case AMP:
          if (thrower.isAmpScoring()) {
            if (!lastIsAmpScoring) {
              ampTimer.restart(); // This timer measures the time since the arm has begun the amp scoring process.
            }
            arm.updateSetpoint(armAmpSetpoint+armAmpRaiseRate*ampTimer.get()); // Raises the arm at 6 deg/sec.
            lastIsAmpScoring = true;
          } else {
            lastIsAmpScoring = false;
            arm.updateSetpoint(armAmpSetpoint);
            thrower.setFlywheelVel(0.0);
          }
          break;

        case MANUAL_SHOOT:
          arm.updateSetpoint(armManualSetpoint);
          thrower.setFlywheelVel(120.0);
          lastIsAmpScoring = false;
          break;

        default:
          break;
      }
    }

    thrower.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the thrower.
    if (thrower.getManualControl()) {
      double flywheelPower = 0.0;
      if (operator.getRawButton(5)) { // Left Bumper
        flywheelPower = 1.0;
      }
      double indexPower = 0.0;
      if (operator.getPOV() == 0) { // D-pad up
        indexPower = 0.3;
      }
      if (operator.getPOV() == 180) { // D-pad down
        indexPower = -0.3;
      }
      thrower.setManualSpeeds(flywheelPower, indexPower);
    } else {
      if (operator.getRawButton(6)) { // Right Bumper
        if (currArmState == ArmState.SHOOT && arm.atSetpoint()) {
          thrower.commandThrow(); // Commands the thrower to throw a note with the commanded flywheel velocity in rotations per second.
        } else if (currArmState == ArmState.AMP && arm.atSetpoint()) {
          thrower.commandAmpScore();
        }
      }
    }

    climber.periodic();
    climber.set(MathUtil.applyDeadband(-operator.getLeftY(), 0.1), MathUtil.applyDeadband(-operator.getRightY(), 0.1));
  }
  
  public void disabledInit() {
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }
  
  public void disabledPeriodic() {
    swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
  }

  // Sends April Tag data to the drivetrain to update the position of the robot on the field. Filters data based on the number of tags visible and their size.
  public void updateVision() {
    boolean isSquare = isSquare();
    SmartDashboard.putBoolean("isSquare", isSquare);
    double[] presentDistanceArray = LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_targetspace").getDoubleArray(new double[6]);
    double presentDistance = -presentDistanceArray[2];
    SmartDashboard.putNumber("Distance to Tag", presentDistance);
    double ta = LimelightHelpers.getTA("");
    double tid = LimelightHelpers.getFiducialID("");
    if (!isSquare && ta > 1.5 && (((tid == 8 || tid == 7) && swerve.isBlueAlliance()) || ((tid == 4 || tid == 3) && swerve.isRedAlliance()))) {
      swerve.addVisionEstimate(0.04, 0.04);
    }
  }
  
  // Determines whether a Limelight target is square. Useful for identifying whether multiple April Tages are detected.
  public boolean isSquare() {
    double thor = LimelightHelpers.getLimelightNTTableEntry("limelight", "thor").getDouble(0);
    double tvert = LimelightHelpers.getLimelightNTTableEntry("limelight", "tvert").getDouble(0);
    if (Math.abs(tvert/thor-1.0) < 0.2) {
      return true;
    } else {
      return false;
    }
  }

  // Calculates the angle the robot should be facing to make the shot in degrees.
  public double getAimHeading() {
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center of the speaker slot in meters, adjusted for alliance. 
    if (swerve.getYPos() == speakerY) { // The robot is aligned with the speaker in the y-dimension. This prevents calls to atan() which would result in undefined returns.
        return 180.0;
    } else if (swerve.getYPos() < speakerY) {
        return Math.atan(swerve.getXPos()/(speakerY-swerve.getYPos()))*180.0/Math.PI + 90.0; // The robot has a positive heading.
    } else {
        return Math.atan(swerve.getXPos()/(speakerY-swerve.getYPos()))*180.0/Math.PI - 90.0; // The robot has a negative heading. 
    }
  }

  // Calcualtes the arm angle that the robot should be at to make the shot. Uses a distance-angle calibration array and linear interpolation.
  private double[] distCalArray = {1.0, 2.0, 3.0}; // Stores the distance between the center of the robot and the center of the speaker in meters. Should be sorted with smallest distances first.
  private double[] armCalArray = {8.0, 21.0, 30.0}; // Stores the arm angle that corresponds with each distance value. This is the angle the arm should be at to make the shot in degrees.
  public double getAimArmAngle() {
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center of the speaker slot in meters, adjusted for alliance. 
    double distToSpeaker = Math.sqrt(Math.pow(speakerY-swerve.getYPos(), 2) + Math.pow(swerve.getXPos(), 2)); // The current distance to the speaker based on the robot's position on the field in meters.
    if (distToSpeaker >= distCalArray[distCalArray.length - 1]) { // If the distance to the speaker is larger than the largest calibration distance.
      return armCalArray[armCalArray.length - 1]; // Return the arm angle that corresponds to the largest calibration distance in the array.
    } else if (distToSpeaker <= distCalArray[0]) { // If the distance to the speaker is smaller than the smallest calibration distance.
      return armCalArray[0]; // Return the arm angle that corresponds to the smallest calibration distance in the array.
    } else { // The distance to the speaker is within the calibration distances tested.
      int lowerIndex = -1; // The index that corresponds to the entry in the calibration array that is immediately smaller than the current robot distance to the speaker.
      for (int i = 0; i < distCalArray.length - 1; i++) { // Iterate through the calibration array, except the last entry.
        if (distCalArray[i+1] > distToSpeaker && lowerIndex == -1) { // Find the first array element that is larger than the current distance to the speaker.
          lowerIndex = i;
        }
      }
      return armCalArray[lowerIndex] + ((armCalArray[lowerIndex+1]-armCalArray[lowerIndex])/(distCalArray[lowerIndex+1]-distCalArray[lowerIndex]))*(distToSpeaker-distCalArray[lowerIndex]); // Linear interpolation
    }
  }

  // Whether the robot is in range to make a shot reliably. 
  public boolean aimShotAvailable() {
    double maxShotDistance = 5.0; // The longest distance that the robot will make a shot from in meters.
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center of the speaker slot in meters, adjusted for alliance. 
    double distToSpeaker = Math.sqrt(Math.pow(speakerY-swerve.getYPos(), 2) + Math.pow(swerve.getXPos(), 2)); // The current distance to the speaker based on the robot's position on the field in meters.
    return distToSpeaker < maxShotDistance;
  }

  // Initializes toggle booleans to the dashboard.
  private boolean moduleToggleFR = false;
  private boolean moduleToggleFL = false;
  private boolean moduleToggleBL = false;
  private boolean moduleToggleBR = false;
  private boolean gyroToggle = false;
  private boolean visionToggle = false;
  private boolean throwerManual = false;
  private boolean throwerReboot = false;
  private boolean armManual = false;
  private boolean armReboot = false;
  private boolean climberReboot = false;
  public void createToggles() {
    SmartDashboard.putBoolean("FR Module Toggle", moduleToggleFR);
    SmartDashboard.putBoolean("FL Module Toggle", moduleToggleFL);
    SmartDashboard.putBoolean("BL Module Toggle", moduleToggleBL);
    SmartDashboard.putBoolean("BR Module Toggle", moduleToggleBR);
    SmartDashboard.putBoolean("Gyro Toggle", gyroToggle);
    SmartDashboard.putBoolean("Vision Toggle", visionToggle);
    SmartDashboard.putBoolean("Thrower Manual Toggle", throwerManual);
    SmartDashboard.putBoolean("Thrower Reboot", throwerReboot);
    SmartDashboard.putBoolean("Arm Manual Toggle", armManual);
    SmartDashboard.putBoolean("Arm Reboot", armReboot);
    SmartDashboard.putBoolean("Climber Reboot", climberReboot);
  }

  // Reads toggles from dashboard and executes the corresponding code.
  public void updateToggles() {
    boolean currModuleToggleFR = SmartDashboard.getBoolean("FR Module Toggle", false);
    if (currModuleToggleFR ^ moduleToggleFR) {
      swerve.toggleFR();
    }
    moduleToggleFR = currModuleToggleFR;

    boolean currModuleToggleFL = SmartDashboard.getBoolean("FL Module Toggle", false);
    if (currModuleToggleFL ^ moduleToggleFL) {
      swerve.toggleFL();
    }
    moduleToggleFL = currModuleToggleFL;

    boolean currModuleToggleBL = SmartDashboard.getBoolean("BL Module Toggle", false);
    if (currModuleToggleBL ^ moduleToggleBL) {
      swerve.toggleBL();
    }
    moduleToggleBL = currModuleToggleBL;

    boolean currModuleToggleBR = SmartDashboard.getBoolean("BR Module Toggle", false);
    if (currModuleToggleBR ^ moduleToggleBR) {
      swerve.toggleBR();
    }
    moduleToggleBR = currModuleToggleBR;

    boolean currGyroToggle = SmartDashboard.getBoolean("Gyro Toggle", false);
    if (currGyroToggle ^ gyroToggle) {
      swerve.toggleGyro();
    }
    gyroToggle = currGyroToggle;

    boolean currVisionToggle = SmartDashboard.getBoolean("Vision Toggle", false);
    if (currVisionToggle ^ visionToggle) {
      swerve.toggleVision();
    }
    visionToggle = currVisionToggle;

    boolean currThrowerManual = SmartDashboard.getBoolean("Thrower Manual Toggle", false);
    if (currThrowerManual ^ throwerManual) {
      thrower.toggleManualControl();
    }
    throwerManual = currThrowerManual;

    boolean currThrowerReboot = SmartDashboard.getBoolean("Thrower Reboot", false);
    if (currThrowerReboot ^ throwerReboot) {
      thrower.reboot();
    }
    throwerReboot = currThrowerReboot;

    boolean currArmManual = SmartDashboard.getBoolean("Arm Manual Toggle", false);
    if (currArmManual ^ armManual) {
      arm.toggleManualControl();
    }
    armManual = currArmManual;

    boolean currArmReboot = SmartDashboard.getBoolean("Arm Reboot", false);
    if (currArmReboot ^ armReboot) {
      arm.reboot();
    }
    armReboot = currArmReboot;

    boolean currClimberReboot = SmartDashboard.getBoolean("Climber Reboot", false);
    if (currClimberReboot ^ climberReboot) {
      climber.reboot();
    }
    climberReboot = currClimberReboot;
  }
}