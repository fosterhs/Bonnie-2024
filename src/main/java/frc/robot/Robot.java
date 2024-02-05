package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick stick = new Joystick(0); // Initializes the joystick.

  // Limits the acceleration of controller inputs. 
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop/Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop/Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngularAccTeleop/Drivetrain.maxAngularVelTeleop);
  
  private final double minSpeedScaleFactor = 0.05; // The maximum speed of the robot when the throttle is at its minimum position, as a percentage of maxVel and maxAngularVel

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

  public void robotInit() {
    // Allows the user to choose which auto to do
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Test", 0.0, 0.0, 0.0, 180.0); // Loads the path. All paths should be loaded in robotInit() because this call is computationally expensive.
    createToggles();

    // Helps prevent loop overruns when the robot is first enabled. These calls cause the robot to initialize code in other parts of the program so it does not need to be initialized during autonomousInit() or teleopInit(), saving computational resources.
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.atPathEndpoint(0);
    swerve.drive(0.1, 0.0, 0.0, false, 0.0, 0.0);
    swerve.resetOdometry(0, 0, 0);
    swerve.updateDash();
  }

  public void robotPeriodic() {
    updateVision(); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    getAim(); // Calculates the required robot heading and arm angle to make a shot from the current robot position.
    swerve.updateDash(); // Pushes drivetrain information to the Dashboard.
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    updateToggles();

    // Re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
    if (stick.getRawButtonPressed(11)) {
      swerve.resetGyro();
    }
  }
  
  public void autonomousInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    thrower.init(); // Must be called during autoInit() and teleopInit() for the thrower to work properly.
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.resetTargetController(180.0);
        break;

      case auto2:
        // AutoInit 2 code goes here.
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
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1: 
            // Auto 1 code goes here.
            swerve.moveToTarget(1.35, 2.6, 180.0);

            // Condition to move to the next stage. The code in the if statement will execute once (like an autoStageInit()), then move on to the next stage.
            if (swerve.atTarget()) {
            thrower.commandThrow(30.0);
            autoStage = 2;
            }
            break;

          default: 
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot after auto is completed.
            break;
        }
        break;

      case auto2:
        // Auto 2 code goes here.
        break;

      case auto3: 
        // Auto 3 code goes here.
        break;

      case auto4: 
        // Auto 4 code goes here.
        break;
    }
  }

  public void teleopInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    thrower.init(); // Must be called during autoInit() and teleopInit() for the thrower to work properly.
  }

  public void teleopPeriodic() {
    double speedScaleFactor = (-stick.getThrottle() + 1 + 2 * minSpeedScaleFactor) / (2 + 2 * minSpeedScaleFactor); // Creates a scale factor for the maximum speed of the robot based on the throttle position.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-stick.getY(), 0.1)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-stick.getX(), 0.1)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-stick.getZ(), 0.1)*speedScaleFactor)*Drivetrain.maxAngularVelTeleop;

    // Auto Rotate to Aim Heading
    double angleDistance = swerve.getAngleDistance(swerve.getFusedAng(), lastAimHeading);
    if (stick.getRawButtonPressed(1)) {
      angleController.reset(angleDistance*Math.PI/180.0, 0.0);
      angleController.setIntegratorRange(-Drivetrain.maxAngularVelAuto*0.8, Drivetrain.maxAngularVelAuto*0.8);
    }
    if (stick.getRawButton(1)) {
      angVel = angleController.calculate(angleDistance*Math.PI/180.0, 0.0);
      if (Math.abs(angleDistance) < headingTol) {
        angVel = 0.0;
      }
      if (Math.abs(angVel) > Drivetrain.maxAngularVelAuto) {
        angVel = angVel > 0.0 ? Drivetrain.maxAngularVelAuto : -Drivetrain.maxAngularVelAuto;
      }
    }

    // Allows the driver to rotate the robot about each corner. Defaults to a center of rotation at the center of the robot.
    if (stick.getRawButton(7)) { // Front Left
      swerve.drive(xVel, yVel, angVel, true, 0.29, 0.29);
    } else if (stick.getRawButton(8)) { // Front Right
      swerve.drive(xVel, yVel, angVel, true, 0.29, -0.29);
    } else if (stick.getRawButton(9)) { // Back Left
      swerve.drive(xVel, yVel, angVel, true, -0.29, 0.29);
    } else if (stick.getRawButton(10)) { // Back Right
      swerve.drive(xVel, yVel, angVel, true, -0.29, -0.29);
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary.
    if (stick.getRawButtonPressed(2)) {
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (stick.getRawButton(2)) {
      swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    }
    if (stick.getRawButtonReleased(2)) {
      swerve.pushCalibration(); // Updates the position of the robot on the field based on previous calculations.
    }

    thrower.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the thrower.
    if (thrower.getManualControl()) {
      thrower.setManualSpeeds(0.0, 0.0); // TODO: Change the inputs to this function to their appropriate keybinds.
    } else {
      if (stick.getRawButton(1)) {
        thrower.commandThrow(30.0); // Commands the thrower to throw a note with a flywheel velocity of 30 rotations per second.
      }
    }

    arm.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the arm.
    if (arm.getManualControl()) {
      arm.setManualPower(0.0); // TODO: Change the inputs to this function to their appropriate keybinds.
    } else {
      arm.updateSetpoint(lastAimArmAngle); // Changes the setpoint of the arm to the calculated arm angle needed to make a shot.
    }

    climber.periodic();
    climber.set(0.0, 0.0); // TODO: Change the inputs to this function to their appropriate keybinds.
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
    if (!isSquare && ta > 1.5 && ((tid == 8 && swerve.isBlueAlliance()) || (tid == 4 && swerve.isRedAlliance()))) {
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

  // This function calculates the required robot heading and arm angle to make a shot into the speaker from the current robot position on the field.
  private final ProfiledPIDController angleController = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Drivetrain.maxAngularVelAuto, Drivetrain.maxAngularAccAuto)); // Controls the angle of the robot.
  double headingTol = 0.5; // The acceptable error in the heading of the robot when tracking headings calculated by the getAim() function.
  boolean lastAimShotAvailable = false; // Whether it is possible to make it into the speaker from the current robot position.
  double lastAimHeading = 0.0; // The robot heading that is required to make the shot.
  double lastAimArmAngle = 0.0; // The arm angle that is required to make the shot.
  public void getAim() {
    double robotX = swerve.getXPos(); // x-coordinate of the center of rotation of the robot in meters.
    double robotY = swerve.getYPos();  // y-coordinate of the center of rotation of the robot in meters.
    double speakerZ = 2.045; // The height of the center of the speaker slot above the field carpet in meters. 
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center of the speaker slot in meters, adjusted for alliance. 
    double armL = 0.6; // The length of the arm between the pivot and the point where the note loses contact with the flywheel in meters.
    double armPivotZ = 0.1; // The height of the arm pivot above the field carpet in meters.
    double armPivotX = 0.2; // The distance from the center of rotation of the robot to the arm pivot in meters. A pivot behind the center of rotation is positive.
    double g = 9.81; // The gravitational acceleration in meters per second squared.
    double noteVel = 8.0; // The velocity of the note as it leaves the thrower in meters per second.
    double minAngle = 10.0; // The lowest angle the arm can expect to shoot at in degrees. 0 degrees is paralell to the floor and 90 degrees is pointing straight up.
    double maxAngle = 80.0; // The highest angle the arm can expect to shoot at in degrees. 0 degrees is paralell to the floor and 90 degrees is pointing straight up.
    int totalAngles = 70; // The total number of angles that should be checked. A higher number takes more computational resources, but is more accurate.
    double[] noteZErrors = new double[totalAngles]; // An array that stores the z-level that the note will impact the speaker at relative to the center of the speaker slot in meters. This represents the overshoot or undershoot error of each angle tested.

    // Calculates the position of the note just as it leaves the thrower. This calculation relies on the previous loops solution to approximate the heading and arm angle. If no shot was available, it will default to using middle of the range values for the heading and arm angle.
    if (!lastAimShotAvailable) {
      lastAimArmAngle = 50.0;
      lastAimHeading = 180.0;
    } 
    double lastNoteX = robotX - armPivotX*Math.cos(lastAimHeading*Math.PI/180.0) + armL*Math.cos(lastAimArmAngle*Math.PI/180.0)*Math.cos(lastAimHeading*Math.PI/180.0); // The trig accounts for the effect of the arm angle and robot heading on the note's position.
    double lastNoteY = robotY + armPivotX*Math.sin(lastAimHeading*Math.PI/180.0) + armL*Math.cos(lastAimArmAngle*Math.PI/180.0)*Math.sin(lastAimHeading*Math.PI/180.0);

    // Calculates the angle the robot should be facing to make the shot in degrees. Based on the position of the note as it leaves the shooter.
    double aimHeading = 180.0; // The angle the robot should be facing to make the shot in degrees.
    if (lastNoteY == speakerY) { // The robot is aligned with the speaker in the y-dimension. This prevents calls to atan() which would result in undefined returns.
        aimHeading = 180.0;
    } else if (lastNoteY < speakerY) {
        aimHeading = Math.atan(lastNoteX/(speakerY-lastNoteY))*180.0/Math.PI + 90.0; // The robot has a positive heading.
    } else {
        aimHeading = Math.atan(lastNoteX/(speakerY-lastNoteY))*180.0/Math.PI - 90.0; // The robot has a negative heading. 
    }
    
    // Calculates the Z-error for several angles to see which angle is the best.
    for (int index = 0; index < totalAngles; index++) {
      double currentAngle = minAngle + index*(maxAngle-minAngle)/totalAngles; // Converts from the array index to degrees.
      double noteX = robotX - armPivotX*Math.cos(aimHeading*Math.PI/180.0) + armL*Math.cos(currentAngle*Math.PI/180.0)*Math.cos(aimHeading*Math.PI/180.0); // The x-coordinate of the note as it leaves contact with the thrower in meters. The trig accounts for the effect of the arm angle and robot heading on the note's position.
      double noteY = robotY + armPivotX*Math.sin(aimHeading*Math.PI/180.0) + armL*Math.cos(currentAngle*Math.PI/180.0)*Math.sin(aimHeading*Math.PI/180.0); // The y-coordinate of the note as it leaves contact with the thrower in meters. The trig accounts for the effect of the arm angle and robot heading on the note's position.
      double noteZ = armL*Math.sin(currentAngle*Math.PI/180.0) + armPivotZ; // The height of the note above the carpet just as it loses contact with the thrower.
      double noteR = Math.sqrt(Math.pow(noteX, 2) + Math.pow(noteY-speakerY, 2)); // The distance between the note's initial position and the speaker slot center.
      double noteRVel = noteVel*Math.cos(currentAngle*Math.PI/180.0); // The radial velocity of the note, as if the speaker slot center was the origin of a polar coordinate system.
      double noteTime = noteR/noteRVel; // The airtime of the note before it impacts the speaker.
      double noteFinalZ =  noteZ + noteVel*Math.sin(currentAngle*Math.PI/180.0)*noteTime - g*Math.pow(noteTime, 2)/2.0; // The kinematics calculated z-position of the note as it impacts the speaker, accounting for intial z velocity and gravity.
      noteZErrors[index] = noteFinalZ - speakerZ; // The error (either undershoot or overshoot) of this arm angle in the z-dimension.
    }
    
    // Looks through the calcuated z-errors to find the correct arm angle to throw the note.
    boolean aimShotAvailable = false; // Stores whether it is physically possible to shoot the note into the speaker from the robot's current position.
    double aimArmAngle = 50.0; // Stores the optimal arm angle to make the shot, or -1 if it is impossible to make the shot.
    for (int index = 0; index < totalAngles-1; index++) {
      if (noteZErrors[index] < 0 && noteZErrors[index+1] > 0) { // There can be two solutions. To identify the correct solution, as the angle increases the Z-error should transition from - to +. The other solution will transition from + to -. If this condition is not met, a shot cannot be made from the robot's current position.
        double negativeAngleZError = -noteZErrors[index];
        double positiveAngleZError = noteZErrors[index+1];
        aimArmAngle = minAngle + (index + negativeAngleZError/(positiveAngleZError + negativeAngleZError))*(maxAngle-minAngle)/totalAngles; // Linear interpolation to approximate the arm angle that results in 0 z-error. 
        aimShotAvailable = true;
      }
    }

    // Stores this periods solution for the next period to iterate on.
    lastAimShotAvailable = aimShotAvailable;
    lastAimHeading = aimHeading;
    lastAimArmAngle = aimArmAngle;

    // Publishes solution to the dashboard.
    SmartDashboard.putNumber("aim robot angle", aimHeading);
    SmartDashboard.putBoolean("aim shotAvailable", aimShotAvailable);
    SmartDashboard.putNumber("aim arm angle", aimArmAngle);
  }

  // Initializes toggle booleans to the dashboard.
  boolean moduleToggleFR = false;
  boolean moduleToggleFL = false;
  boolean moduleToggleBL = false;
  boolean moduleToggleBR = false;
  boolean gyroToggle = false;
  boolean visionToggle = false;
  boolean throwerManual = false;
  boolean throwerReboot = false;
  boolean armManual = false;
  boolean armReboot = false;
  boolean climberReboot = false;
  public void createToggles() {
    moduleToggleFR = false;
    moduleToggleFL = false;
    moduleToggleBL = false;
    moduleToggleBR = false;
    gyroToggle = false;
    visionToggle = false;
    throwerManual = false;
    throwerReboot = false;
    armManual = false;
    armReboot = false;
    climberReboot = false;

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