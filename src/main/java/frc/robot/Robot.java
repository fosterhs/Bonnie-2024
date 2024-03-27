package frc.robot;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.
  private final XboxController operator = new XboxController(1); // Initializes the operator controller.

  // Limits the acceleration of controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(
      Drivetrain.maxAngularAccTeleop / Drivetrain.maxAngularVelTeleop);

  private double speedScaleFactor = 1.0; // Scales the speed of the robot that results from controller inputs. 1.0
                                         // corresponds to full speed. 0.0 is fully stopped.

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target
                                                      // Tracking, Odometry, and Vision Calibration.
  private final Thrower thrower = new Thrower();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();

  // Auto Chooser Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "1 Piece Mid";
  private static final String auto2 = "2 Piece";
  private static final String auto3 = "3 Piece";
  private static final String auto4 = "CLIMBER_RESET";
  private static final String auto5 = "Auto 5 Amp(?)";
  private static final String auto6 = "Auto 6 (?)";
  private static final String auto7 = "1 Piece Side";
  private static final String auto8 = "4 piece (WIP)";
  private static final String auto9 = "4 piece (Zone)";
  private static final String auto10 = "2 Piece Side ";
  private static final String auto11 = "Troll Auto (Right)";
  private String autoSelected;
  private int autoStage = 1;
  private boolean lastIsAmpScoring = false; // Stores whether the thrower was amp scoring in the previous period.
  private final Timer ampTimer = new Timer(); // Controls the inclination of the arm during amp scoring.

  private final CANdle candle0 = new CANdle(0, "rio"); // Initialzes the LEDs on the left.
  private final CANdle candle1 = new CANdle(1, "rio"); // Initialzes the LEDs on the right.
  private boolean lightsOn = false;
  private int noteIterations = 0;
  private int strobeIterations = 0;

  private Timer rumbleTimer = new Timer(); // Duration of rumble intake cue
  private Timer noteFiredTimer = new Timer(); // Tracks time elapsed since note fired
  private boolean hadNote = false; // Tracks if the robot previously had a note

  // Arm States (Teleop)
  private enum ArmState {
    DRIVE,
    SHOOT,
    AMP,
    INTAKE,
    MANUAL_SHOOT;
  }

  ArmState currArmState = ArmState.INTAKE; // Stores the current arm state. The robot will default to the value
                                          // intialized here when teleop is first entered.
  private final double armDriveSetpoint = 75.0; // The arm's driving position in degrees.
  private final double armAmpSetpoint = 48.0; // The arm's inital amp scoring position in degrees.
  private final double armIntakeSetpoint = -5.0; // The arm's intake position in degrees.
  private final double armAmpRaiseRate = 6.0; // The rate at which the arm is raised during amp scoring in deg/sec.
  private final double armManualSetpoint = 8.0; // THe arm's manual shooting position in degrees.
  private final Timer armTimer = new Timer(); // Tracks the number of secound that the arm is at the setpoint

  private double armDashControl = 75.0;
  private boolean rightTriggerWasPressed = false;
  private boolean leftTriggerWasPressed = false;

  public void robotInit() {
    // Allows the user to choose which auto to do
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    autoChooser.addOption(auto4, auto4);
    autoChooser.addOption(auto5, auto5);
    autoChooser.addOption(auto6, auto6);
    autoChooser.addOption(auto7, auto7);
    autoChooser.addOption(auto8, auto8);
    autoChooser.addOption(auto9, auto9);
    autoChooser.addOption(auto10, auto10);
    autoChooser.addOption(auto11, auto11);
    SmartDashboard.putData("Autos", autoChooser);

    ampTimer.restart(); // Gets the amp timer started. Used in teleop to incline the arm.
    armTimer.restart(); // Gets the arm timer started.

    createToggles(); // Creates the infrastructure for using dashboard toggles.
    SmartDashboard.putNumber("Arm Dash Control", armDashControl);

    swerve.loadPath("Rush Center", 0.0, 0.0, 0.0, 120.0); // Loads the path. All paths should be loaded in robotInit() because this call is computationally expensive.
    swerve.loadPath("Return From Center", 0.0, 0.0, 0.0, 180.0);

    // Helps prevent loop overruns when the robot is first enabled. These calls
    // cause the robot to initialize code in other parts of the program so it does
    // not need to be initialized during autonomousInit() or teleopInit(), saving
    // computational resources.
    swerve.resetDriveController(0.0);
    swerve.aimDrive(0.01, 0.0, 0.0, true);
    swerve.driveTo(0.0, 0.0, 0.0);
    swerve.addCalibrationEstimate();
    swerve.pushCalibration();
    swerve.resetCalibration();
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.atPathEndpoint(0);
    swerve.drive(0.01, 0.0, 0.0, false, 0.0, 0.0);
    swerve.resetOdometry(0, 0, 0);
    swerve.updateDash();
    climber.setManual(0.0, 0.0);
    arm.atSetpoint();
    arm.periodic();
    arm.updateSetpoint(armDriveSetpoint);
    arm.setManualPower(0.0);
    thrower.init();
    thrower.periodic();
    robotPeriodic();
  }

  public void robotPeriodic() {
    swerve.updateDash(); // Pushes drivetrain information to the Dashboard.
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each
                             // period.
    arm.updateDashboard();
    thrower.updateDashboard();
    climber.updateDashboard();
    updateToggles(); // Checks the dashboard toggles and takes any actions based on them.
    SmartDashboard.putNumber("autoStage", autoStage);
    SmartDashboard.putNumber("ArmTimer", armTimer.get());
    armDashControl = SmartDashboard.getNumber("Arm Dash Control", 75.0);
    controlLEDs();

    if (!arm.atSetpoint()) { // Resets the arm timer to 0 if the arm is not at the current setpoint.
      armTimer.restart();
    }

    // Re-zeros the angle reading of the gyro to the current angle of the robot.
    // Should be called if the gyroscope readings are no longer well correlated with
    // the field.
    if (driver.getRawButtonPressed(8)) {
      swerve.resetGyro();
    }
  }

  public void autonomousInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    thrower.init(); // Must be called during autoInit() and teleopInit() for the thrower to work
                    // properly.
    armTimer.restart();
    climber.init();
    arm.init();
    autoStage = swerve.isCalibrated() && !arm.getMotorFailure() && !thrower.getMotorFailure()
        && !swerve.getModuleFailure() && !swerve.getGyroFailure() ? 1 : -1; // Goes to default case if April Tags were
                                                                            // not visible.
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;

      case auto3:
        // AutoInit 3 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;

      case auto4:
        // AutoInit 4 code goes here.
        arm.updateSetpoint(armIntakeSetpoint);
        thrower.setDisableFlywheel(true);
        break;

      case auto5:
        // AutoInit 5 code goes here.
        swerve.resetDriveController(getAimHeading());
        break;

      case auto6:
        // AutoInit 6 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;

      case auto7:
        // AutoInit 7 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;

      case auto8:
        // AutoInit 8 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;
      case auto9:
        // AutoInit 9 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;
      case auto10:
        // AutoInit 10 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;

      case auto11:
        // AutoInit 11 code goes here.
        swerve.resetDriveController(getAimHeading());
        arm.updateSetpoint(getAimArmAngle());
        thrower.setDisableFlywheel(false);
        break;
    }
  }

  public void autonomousPeriodic() {
    thrower.periodic();
    arm.periodic();
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1 code goes here.
            swerve.driveTo(1.91, swerve.isBlueAlliance() ? 5.48 : Drivetrain.fieldWidth - 5.48, getAimHeading());
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                armTimer.restart();
                arm.updateSetpoint(armDriveSetpoint);
                autoStage = -1; // Goes to default case.
              }
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot after auto is completed.
            arm.updateSetpoint(armDriveSetpoint);
            break;
        }
        break;

      case auto2:
        // Auto 2 code goes here.
        switch (autoStage) {
          case 1:
            swerve.driveTo(1.91, swerve.isBlueAlliance() ? 5.48 : Drivetrain.fieldWidth - 5.48, getAimHeading());
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { 
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
              swerve.resetDriveController(getAimHeading());
              arm.updateSetpoint(getAimArmAngle());
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
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                autoStage = -1; // Default case
              }
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, false, 0, 0);
            arm.updateSetpoint(armDriveSetpoint);
            break;
        }
        break;

      case auto3:
        // Auto 3 code goes here.
        switch (autoStage) {
          case 1:
            swerve.aimDrive(0.0, 0.0, getAimHeading(), true);
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                swerve.resetPathController(0);
                autoStage = 2;
              }
            }
            break;

          case 2:
            swerve.followPath(0);

            if (armTimer.get() > 2.5) {
              arm.updateSetpoint(armIntakeSetpoint);
            }

            if (thrower.getSensor1()) {
              swerve.resetPathController(1);
              arm.updateSetpoint(getAimArmAngle());
              autoStage = 3;
            }
            break;

          case 3:
            swerve.followPath(1);
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atPathEndpoint(1) && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                arm.updateSetpoint(armDriveSetpoint);
                autoStage = -1; // Default case
              }
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0);
            arm.updateSetpoint(armDriveSetpoint);
            break;
        }
        break;

      case auto4:
        // Auto 4 code goes here.
        swerve.drive(0.0, 0.0, 0.0, false, 0, 0);
        climber.disableLockout();
        climber.setManual(-0.3, -0.3);
        if (climber.getLeftSensor() && climber.getRightSensor()) {
          arm.updateSetpoint(armDriveSetpoint);
        }
        break;

      case auto5:
        // Auto 5 code goes here.
        switch (autoStage) {
          case 1:
            swerve.driveTo(2.0, (swerve.isBlueAlliance() ? 7.5 : Drivetrain.fieldWidth - 7.5),
                (swerve.isBlueAlliance() ? -90.0 : 90.0));
            arm.updateSetpoint(armAmpSetpoint);
            thrower.setDisableFlywheel(true);

            if (swerve.atDriveGoal() && arm.atSetpoint()) {
              thrower.commandAmpScore();
              if (!thrower.isAmpScoring() && !thrower.getSensor1() && !thrower.getSensor2() && !thrower.getSensor3()) {
                swerve.resetDriveController(0.0);
                autoStage = 2;
              }
            }
            break;

          case 2:
            swerve.driveTo(3.5, (swerve.isBlueAlliance() ? 7.5 : 0.5), 0.0);
            arm.updateSetpoint(armDriveSetpoint);
            if (swerve.atDriveGoal()) {
              swerve.resetDriveController(0.0);
              autoStage = -1;
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
            arm.updateSetpoint(armDriveSetpoint);
            break;
        }

      case auto6:
        // Auto 6 code goes here.
        switch (autoStage) {
          case 1:
            swerve.aimDrive(0.0, 0.0, getAimHeading(), true);
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 2; // Goes to default case.
              }
            }
            break;

          case 2:
            // 6.51 , 0.98
            arm.updateSetpoint(armIntakeSetpoint);
            swerve.driveTo(6.95, (swerve.isBlueAlliance() ? 0.80 : Drivetrain.fieldWidth - 0.80), 180.0);
            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
              armTimer.restart();
              arm.updateSetpoint(armIntakeSetpoint);
              swerve.resetDriveController(180.0);
              autoStage = 3;
            }
          break;

          case 3:

          break;

          default:
            arm.updateSetpoint(armDriveSetpoint);
            swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
            break;
        }
        break;

      case auto7:
        switch (autoStage) {
          case 1:
            // Auto 7 code goes here
            arm.updateSetpoint(getAimArmAngle()); // 1.61, 6.68
            swerve.aimDrive(0.0, 0.0, getAimHeading(), true);
            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                armTimer.restart();
                arm.updateSetpoint(armDriveSetpoint);
                autoStage = -1; // Goes to default case.
              }
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, false, 0.0, 0.0); // Stops the robot after auto is completed.
            arm.updateSetpoint(armDriveSetpoint);
            break;
        }
        break;

      case auto8:
        switch (autoStage) {
          case 1:
          swerve.driveTo(swerve.getXPos(), swerve.getYPos(), getAimHeading());
          arm.updateSetpoint(getAimArmAngle());

          if (swerve.atDriveGoal() && arm.atSetpoint() && aimShotAvailable() && armTimer.get() > 0.5) {
            thrower.commandThrow();

            if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2() && !thrower.getSensor3()) {
              armTimer.restart();
              arm.updateSetpoint(armIntakeSetpoint);
              swerve.resetDriveController(180.0);
              autoStage = 2; // Goes to default case.
            }
          }

        break;


        case 2:
        swerve.driveTo(swerve.getXPos(), swerve.getYPos(), 180.0);

          if (armTimer.get() > 0.3) {
            swerve.resetDriveController(0.0);
            autoStage = 3;
          }

        break;


        case 3:
        swerve.aimDrive(1.0, 0.0, 180.0, true);

          if (thrower.getSensor1()) {
            swerve.resetDriveController(getAimHeading());
            arm.updateSetpoint(getAimArmAngle());
            armTimer.restart();
            autoStage = 4;
          } else if (swerve.getXPos() > 4.0) {
            autoStage = -1; // Goes to default case.
          }

        break;

        case 4: 
         swerve.driveTo(1.75, (swerve.isBlueAlliance() ? 3.72 : Drivetrain.fieldWidth - 3.72), getAimHeading());
          arm.updateSetpoint(getAimArmAngle());

          if (swerve.atDriveGoal() && arm.atSetpoint() && aimShotAvailable() && armTimer.get() > 0.5) {
            thrower.commandThrow();

            if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2() && !thrower.getSensor3()) {
              armTimer.restart();
              arm.updateSetpoint(armDriveSetpoint);
              swerve.resetDriveController(180.0);
              autoStage = 5;
            }
          }

        break;

        case 5:
        swerve.driveTo(2.15, (swerve.isBlueAlliance() ? 5.58 : Drivetrain.fieldWidth - 5.58), 180);
        arm.updateSetpoint(armIntakeSetpoint);

        if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
          armTimer.restart();
          swerve.resetDriveController(180.0);
          autoStage = 6;
        }

        break;


        case 6:
        swerve.aimDrive(1.0, 0.0, 180.0, true);

          if (thrower.getSensor1()) {
            swerve.resetDriveController(getAimHeading());
            arm.updateSetpoint(getAimArmAngle());
            armTimer.restart();
            autoStage = 7;
          } else if (swerve.getXPos() > 4.0) {
            autoStage = -1; 
          }

        break;


        case 7:
        swerve.driveTo(1.9, (swerve.isBlueAlliance() ? 3.72 : Drivetrain.fieldWidth - 3.72), getAimHeading());
          arm.updateSetpoint(getAimArmAngle());

          if (swerve.atDriveGoal() && arm.atSetpoint() && aimShotAvailable() && armTimer.get() > 0.5) {
            thrower.commandThrow();

            if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2() && !thrower.getSensor3()) {
              armTimer.restart();
              arm.updateSetpoint(armDriveSetpoint);
              swerve.resetDriveController(180.0);
              autoStage = 8;
            }
          }

        break;


        case 8:
        swerve.driveTo(7.6, (swerve.isBlueAlliance() ? 0.75 : Drivetrain.fieldWidth - 0.75), 180.0);
        if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.5) {
          armTimer.restart();
          swerve.resetDriveController(180.0);
          autoStage = 9;
        }

        break;

        case 9:
          swerve.aimDrive(1.0, 0.0, 180.0, true);

          if (thrower.getSensor1()) {
            swerve.resetDriveController(getAimHeading());
            arm.updateSetpoint(getAimArmAngle());
            armTimer.restart();
            autoStage = 10;
          } else if (swerve.getXPos() > 4.0) {
            autoStage = -1; 
          }

        break;


        case 10:
          swerve.driveTo(1.9, (swerve.isBlueAlliance() ? 3.72 : Drivetrain.fieldWidth - 3.72), getAimHeading());
          arm.updateSetpoint(getAimArmAngle());

          if (swerve.atDriveGoal() && arm.atSetpoint() && aimShotAvailable() && armTimer.get() > 0.5) {
            thrower.commandThrow();

            if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2() && !thrower.getSensor3()) {
              armTimer.restart();
              arm.updateSetpoint(armDriveSetpoint);
              swerve.resetDriveController(180.0);
              autoStage = 11;
            }
          }
        break;


        case 11:
        swerve.driveTo(swerve.getXPos() + 2.0, swerve.getYPos(), 180);
        if (swerve.atDriveGoal()) {
          swerve.resetDriveController(180);
          autoStage = -1;
        }
        break;

        default:
          swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
          break;
        }
        break;
      case auto9:
        switch (autoStage) {
          case 1: //Put robot at shooting pos
            swerve.driveTo(1.91, swerve.isBlueAlliance() ? 5.48 : Drivetrain.fieldWidth - 5.48, getAimHeading());
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.0) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once ( like an autoStageInit() ), then move on to the next stage.
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 2;
              } 
            }          
            break;
          
          case 2:
          swerve.aimDrive(0.0, 0.0, 180.0, true);

            if (armTimer.get() > 0) {
              autoStage = 3;
            }
            break;

          case 3: // gets the midile pice 
            swerve.aimDrive(1.0, 0.0, 180.0, true);

            if (thrower.getSensor1()) {
              swerve.resetDriveController(getAimHeading());
              arm.updateSetpoint(getAimArmAngle());
              armTimer.restart();
              autoStage = 4;
            } else if (swerve.getXPos() > 4.0) {
              autoStage = -1; // Goes to default case.
            } 
            break;
          
          case 4:
            swerve.driveTo(1.91, swerve.isBlueAlliance() ? 5.48 : Drivetrain.fieldWidth - 5.48, getAimHeading());
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.0) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once ( like an autoStageInit() ), then move on to the next stage.
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 5;
              } 
            }
            break;
          // Add delay if needed
          case 5: // Gwt top pice
            swerve.driveTo(2.67, swerve.isBlueAlliance() ? 6.65 : Drivetrain.fieldWidth - 6.65, swerve.isBlueAlliance() ? -120.0 : 120.0);

            if (thrower.getSensor1()) {
              swerve.resetDriveController(getAimHeading());
              arm.updateSetpoint(getAimArmAngle());
              armTimer.restart();
              autoStage = 6;
            } else if (swerve.getXPos() > 4.0) {
              autoStage = -1; // Goes to default case.
            }
            break;
          
          case 6:
            swerve.driveTo(1.91, swerve.isBlueAlliance() ? 5.48 : Drivetrain.fieldWidth - 5.48, getAimHeading());
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.0) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once ( like an autoStageInit() ), then move on to the next stage.
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 7;
              } 
            }
            break;
          
          case 7:// gets bottom pice
            swerve.driveTo(2.67, swerve.isBlueAlliance() ? 4.36 : Drivetrain.fieldWidth - 4.36, swerve.isBlueAlliance() ? 120 : -120);

            if (thrower.getSensor1()) {
              swerve.resetDriveController(getAimHeading());
              arm.updateSetpoint(getAimArmAngle());
              armTimer.restart();
              autoStage = 8;
            }
            break;

          case 8:
            swerve.driveTo(1.91, swerve.isBlueAlliance() ? 5.48 : Drivetrain.fieldWidth - 5.48, getAimHeading());
            arm.updateSetpoint(getAimArmAngle());


            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the next stage. The code in the if statement will execute once ( like an autoStageInit() ), then move on to the next stage.
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 9;
              }
            }
            break;

          case 9: // Center note
            swerve.driveTo(8.7, (swerve.isBlueAlliance() ? 7.3 : Drivetrain.fieldWidth - 7.3), 180.0);

            if (swerve.atDriveGoal()) {
              arm.updateSetpoint(armDriveSetpoint);
              autoStage = -1;
            }
          break;

          default:
            swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
            break;
        }
        break;

      case auto10:
        switch (autoStage) {
          case 1:
            arm.updateSetpoint(getAimArmAngle());

            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { 
                armTimer.restart();
                arm.updateSetpoint(armIntakeSetpoint);
                swerve.resetDriveController(180.0);
                autoStage = 2;
              }
            }
            break;

          case 2:
            swerve.aimDrive(0.0, 0.0, 180.0, true);

            if (armTimer.get() > 0.2) {
              autoStage = 3;
            }
            break;

          case 3:
            swerve.aimDrive(1.0, 0.0, 180.0, true);

            if (thrower.getSensor1()) {
              swerve.resetDriveController(getAimHeading());
              arm.updateSetpoint(getAimArmAngle());
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
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { // Condition to move to the
                                                                                             // next stage. The code in
                                                                                             // the if statement will
                                                                                             // execute once (like an
                                                                                             // autoStageInit()), then
                                                                                             // move on to the next
                                                                                             // stage.
                autoStage = -1; // Default case
              }
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, false, 0, 0);
            arm.updateSetpoint(armDriveSetpoint);
            break;
        }
        break;

      case auto11:
        switch (autoStage) {
          case 1:
            swerve.driveTo(swerve.getXPos(), swerve.getYPos(), getAimHeading());
            arm.updateSetpoint(getAimArmAngle());
            
            if (swerve.atDriveGoal() && arm.atSetpoint() && armTimer.get() > 0.3) {
              thrower.commandThrow();
              if (!thrower.isThrowing() && !thrower.getSensor1() && !thrower.getSensor2()) { 
                armTimer.restart();
                  arm.updateSetpoint(armDriveSetpoint);
                  swerve.resetDriveController(45.0);
                  autoStage = 2;
                }
              }
              break;

          case 2:
            swerve.driveTo(7.75, (swerve.isBlueAlliance() ? 0.73 : Drivetrain.fieldWidth - 0.73), 180.0);
            if (swerve.atDriveGoal()) {
              swerve.resetDriveController(45.0);
              autoStage = 3;
            }
            break;

          case 3:
            swerve.driveTo(7.75, (swerve.isBlueAlliance() ? 7.0 : Drivetrain.fieldWidth - 7.0), (swerve.isBlueAlliance() ? 45.0 : -45.0)); 
            if (swerve.atDriveGoal()) {
              swerve.resetDriveController(180.0);
              autoStage = -1;
            }
            break;

          default:
            swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
            arm.updateSetpoint(armDriveSetpoint);
            break;             
        }
        break;

      default:
        swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
        arm.updateSetpoint(armDriveSetpoint);
        break;
    }
  }
  

  public void teleopInit() {
    double ta = LimelightHelpers.getTA("");
    if (ta > 0.01) {
      swerve.pushCalibration(); // Updates the robot's position on the field.
    }
    thrower.init(); // Must be called during autoInit() and teleopInit() for the thrower to work properly.
    climber.init();
    // arm.init();
    rumbleTimer.restart();
    noteFiredTimer.restart();
  }

  public void teleopPeriodic() {
    updateVision(); // Checks to see ifs there are reliable April Tags in sight of the Limelight and
                // updates the robot position on the field.
    if (driver.getRawButtonPressed(4)) { // Y Button
      speedScaleFactor = 1.0;
    }
    if (driver.getRawButtonPressed(2)) { // B button
      speedScaleFactor = 0.6;
    }
    if (driver.getRawButtonPressed(1)) { // A button
      speedScaleFactor = 0.15;
    }

    // Applies a deadband to controller inputs. Also limits the acceleration of
    // controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05) * speedScaleFactor)
        * Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05) * speedScaleFactor)
        * Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05) * speedScaleFactor)
        * Drivetrain.maxAngularVelTeleop;

    // Auto Rotate to Aim Heading
    boolean rightTriggerPressed = driver.getRightTriggerAxis() > 0.25;
    boolean leftTriggerPressed = driver.getLeftTriggerAxis() > 0.25;
    if (driver.getRawButtonPressed(6)) { // Right Bumper
      swerve.resetDriveController(getAimHeading());
    } else if (driver.getRawButtonPressed(5)) { // Left Bumper
      swerve.resetDriveController(swerve.isBlueAlliance() ? -90.0 : 90.0); // Rotate to amp.
    } else if (rightTriggerPressed && !rightTriggerWasPressed) {
      swerve.resetDriveController(getAimHeading());
    } else if (leftTriggerPressed && !leftTriggerWasPressed) {
      swerve.resetDriveController(swerve.isBlueAlliance() ? -90.0 : 90.0); // Rotate to amp.
    }
    rightTriggerWasPressed = rightTriggerPressed;
    leftTriggerWasPressed = leftTriggerPressed; 

    if (driver.getRawButton(6)) { // Right Bumper
      swerve.driveTo(1.89, (swerve.isBlueAlliance() ? 5.56 : Drivetrain.fieldWidth - 5.56), getAimHeading()); // Snap to speaker.
    } else if (driver.getRawButton(5)) { // Left Bumper
      swerve.driveTo(1.8, (swerve.isBlueAlliance() ? 7.42 : Drivetrain.fieldWidth - 7.42), (swerve.isBlueAlliance() ? -90.0 : 90.0)); // Snap to amp.
    } else if (rightTriggerPressed) {
      swerve.aimDrive(xVel, yVel, getAimHeading(), true);
    } else if (leftTriggerPressed) {
      swerve.aimDrive(xVel, yVel, swerve.isBlueAlliance() ? -90.0 : 90.0, true);
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drives the robot at a certain speed and rotation rate. Units: meters per second for xVel and yVel, radians per second for angVel.
    }

    // The following 3 calls allow the user to calibrate the position of the robot
    // based on April Tag information. Should be called when the robot is
    // stationary.
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
    if (climber.getLockout()) { // Climber is not active.
      if (arm.getManualControl()) {
        arm.setManualPower(operator.getRightTriggerAxis() * 0.25 - operator.getLeftTriggerAxis() * 0.25);
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
            thrower.setDisableFlywheel(true);
            lastIsAmpScoring = false;
            break;

          case DRIVE:
            arm.updateSetpoint(armDriveSetpoint);
            thrower.setDisableFlywheel(true);
            lastIsAmpScoring = false;
            break;

          case SHOOT:
            arm.updateSetpoint(getAimArmAngle());
            thrower.setDisableFlywheel(false);
            lastIsAmpScoring = false;
            break;

          case AMP:
            if (thrower.isAmpScoring()) {
              if (!lastIsAmpScoring) {
                ampTimer.restart(); // This timer measures the time since the arm has begun the amp scoring process.
              }
              arm.updateSetpoint(armAmpSetpoint + armAmpRaiseRate * ampTimer.get()); // Raises the arm at 6 deg/sec.
              lastIsAmpScoring = true;
            } else {
              lastIsAmpScoring = false;
              arm.updateSetpoint(armAmpSetpoint);
              thrower.setDisableFlywheel(true);
            }
            break;

          case MANUAL_SHOOT:
            arm.updateSetpoint(armManualSetpoint);
            thrower.setDisableFlywheel(false);
            lastIsAmpScoring = false;
            break;

          default:
            break;
        }
      }
    }

    boolean hasNote = thrower.getSensor1() || thrower.getSensor2() || thrower.getSensor3(); // Rumble cue when the robot
                                                                                            // intakes a note
    if ((hasNote && !hadNote) || (!hasNote && hadNote)) { // Note Pickup Rumble Cue
      rumbleTimer.restart();
      driver.setRumble(RumbleType.kBothRumble, 0.2);
      operator.setRumble(RumbleType.kBothRumble, 0.2);
    }
    if (rumbleTimer.get() > 0.4) {
      driver.setRumble(RumbleType.kBothRumble, 0.0);
      operator.setRumble(RumbleType.kBothRumble, 0.0);
    }
    hadNote = hasNote;

    thrower.periodic(); // Should be called in teleopPeriodic() and autoPeriodic(). Handles the internal logic of the thrower.
    if (climber.getLockout()) {
      if (thrower.getManualControl()) {
        double flywheelPower = 0.0;
        if (operator.getRawButton(5)) { // Left Bumper
          flywheelPower = 1.0;
        }
        double indexPower = 0.0;
        if (operator.getPOV() == 0) { // D-pad up
          indexPower = 1.0;
        }
        if (operator.getPOV() == 180) { // D-pad down
          indexPower = -1.0;
        }
        thrower.setManualSpeeds(flywheelPower, indexPower);
      } else {
        if (operator.getRawButton(6)) { // Right Bumper
          if (arm.getManualControl()) {
            thrower.commandThrow();
          } else if (currArmState == ArmState.SHOOT && arm.atSetpoint()) {
            thrower.commandThrow(); // Commands the thrower to throw a note with the commanded flywheel velocity in rotations per second.
          } else if (currArmState == ArmState.AMP && arm.atSetpoint()) {
            thrower.commandAmpScore();
          }
        }
      }
    } else {
      thrower.setDisableFlywheel(false);
    }

    if (operator.getRawButtonPressed(7) && arm.getArmEncoderLeft() < 10.0) { // Mode Button
      climber.disableLockout();
    }
    climber.setManual(MathUtil.applyDeadband(-operator.getLeftY(), 0.1),
        MathUtil.applyDeadband(-operator.getRightY(), 0.1));
  }

  public void disabledInit() {    
    double ta = LimelightHelpers.getTA("");
    if (ta > 0.01) {
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
  }

  public void disabledPeriodic() {
    double ta = LimelightHelpers.getTA("");
    if (ta > 0.01) {
      swerve.addCalibrationEstimate(); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    }
  }

  // Sets the LEDs based on whether a note is detected.
  public void controlLEDs() {
    boolean hasNote = thrower.getSensor1() || thrower.getSensor2() || thrower.getSensor3();
    boolean armFailure = arm.getMotorFailure() || !arm.isCalibrated();
    boolean climberFaliure = climber.getMotorFailure() || !arm.isCalibrated();
    boolean swerveFailure = swerve.getGyroFailure() || swerve.getGyroDisabled() || swerve.getModuleFailure()
        || swerve.getModuleDisabled() || swerve.getVisionDisconnected() || swerve.getVisionDisabled()
        || !swerve.isCalibrated();

    if (armFailure || climberFaliure || swerveFailure || thrower.getMotorFailure()) {
      noteIterations = 0;
      strobeIterations = 0;
      lightsOn = false;

      candle0.setLEDs(255, 0, 0, 0, 0, 8);
      candle1.setLEDs(255, 0, 0, 0, 0, 8);
    } else if (hasNote) {
      if (noteIterations % 4 == 0 && strobeIterations < 11) {
        lightsOn = !lightsOn;
        strobeIterations++;
      }
      noteIterations++;
      if (lightsOn) {
        candle0.setLEDs(0, 255, 0, 0, 0, 8);
        candle1.setLEDs(0, 255, 0, 0, 0, 8);
      } else {
        candle0.setLEDs(0, 0, 0, 0, 0, 8);
        candle1.setLEDs(0, 0, 0, 0, 0, 8);
      }
    } else {
      noteIterations = 0;
      strobeIterations = 0;
      lightsOn = false;

      candle0.setLEDs(255, 0, 255, 0, 0, 8);
      candle1.setLEDs(255, 0, 255, 0, 0, 8);
    }
  }

  // Sends April Tag data to the drivetrain to update the position of the robot on
  // the field. Filters data based on the number of tags visible and their size.
  public void updateVision() {
    boolean isSquare = isSquare();
    SmartDashboard.putBoolean("isSquare", isSquare);
    double[] presentDistanceArray = LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_targetspace")
        .getDoubleArray(new double[6]);
    double presentDistance = -presentDistanceArray[2];
    SmartDashboard.putNumber("Distance to Tag", presentDistance);
    double ta = LimelightHelpers.getTA("");
    if (!isSquare && ta > 1.5 && swerve.getXVel() < 0.1 && swerve.getYVel() < 0.1 && swerve.getAngVel() < 0.1) {
      swerve.addVisionEstimate(0.04, 0.04);
    } else {
    }
  }

  // Determines whether a Limelight target is square. Useful for identifying
  // whether multiple April Tages are detected.
  public boolean isSquare() {
    double thor = LimelightHelpers.getLimelightNTTableEntry("limelight", "thor").getDouble(0);
    double tvert = LimelightHelpers.getLimelightNTTableEntry("limelight", "tvert").getDouble(0);
    if (Math.abs(tvert / thor - 1.0) < 0.2) {
      return true;
    } else {
      return false;
    }
  }

  // Calculates the angle the robot should be facing to make the shot in degrees.
  public double getAimHeading() {
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center of the speaker slot in meters, adjusted for alliance.
    if (swerve.getYPos() == speakerY) { // The robot is aligned with the speaker in the y-dimension. This prevents calls
                                        // to atan() which would result in undefined returns.
      return 180.0;
    } else if (swerve.getYPos() < speakerY) {
      return Math.atan(swerve.getXPos() / (speakerY - swerve.getYPos())) * 180.0 / Math.PI + 90.0; // The robot has a
                                                                                                   // positive heading.
    } else {
      return Math.atan(swerve.getXPos() / (speakerY - swerve.getYPos())) * 180.0 / Math.PI - 90.0; // The robot has a
                                                                                                   // negative heading.
    }
  }

  // getAimNoteHeading() (WIP)
  // Switch to Limelight neural network
  // Calculate robot heading
  // Rotate
  // Switch out of Limelight neural network
  public double getAimNoteHeading() { // Rotates towards note
    return 0.0;
  }

  // Calcualtes the arm angle that the robot should be at to make the shot. Uses a
  // distance-angle calibration array and linear interpolation.
  private double[] distCalArray = { 1.58, 2.25, 2.75 }; // Stores the distance between the center of the robot and the
                                                        // center of the speaker in meters. Should be sorted with
                                                        // smallest distances first.
  private double[] armCalArray = { -4.00, 4.80, 13.50 }; // Stores the arm angle that corresponds with each distance
                                                         // value. This is the angle the arm should be at to make the
                                                         // shot in degrees.

  public double getAimArmAngle() {
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center
                                                                                       // of the speaker slot in meters,
                                                                                       // adjusted for alliance.
    double distToSpeaker = Math.sqrt(Math.pow(speakerY - swerve.getYPos(), 2) + Math.pow(swerve.getXPos(), 2)); // The
                                                                                                                // current
                                                                                                                // distance
                                                                                                                // to
                                                                                                                // the
                                                                                                                // speaker
                                                                                                                // based
                                                                                                                // on
                                                                                                                // the
                                                                                                                // robot's
                                                                                                                // position
                                                                                                                // on
                                                                                                                // the
                                                                                                                // field
                                                                                                                // in
                                                                                                                // meters.
    SmartDashboard.putNumber("Distance to Speaker", distToSpeaker);
    if (distToSpeaker >= distCalArray[distCalArray.length - 1]) { // If the distance to the speaker is larger than the
                                                                  // largest calibration distance.
      return armCalArray[armCalArray.length - 1]; // Return the arm angle that corresponds to the largest calibration
                                                  // distance in the array.
    } else if (distToSpeaker <= distCalArray[0]) { // If the distance to the speaker is smaller than the smallest
                                                   // calibration distance.
      return armCalArray[0]; // Return the arm angle that corresponds to the smallest calibration distance in
                             // the array.
    } else { // The distance to the speaker is within the calibration distances tested.
      int lowerIndex = -1; // The index that corresponds to the entry in the calibration array that is
                           // immediately smaller than the current robot distance to the speaker.
      for (int i = 0; i < distCalArray.length - 1; i++) { // Iterate through the calibration array, except the last
                                                          // entry.
        if (distCalArray[i + 1] > distToSpeaker && lowerIndex == -1) { // Find the first array element that is larger
                                                                       // than the current distance to the speaker.
          lowerIndex = i;
        }
      }
      return armCalArray[lowerIndex] + ((armCalArray[lowerIndex + 1] - armCalArray[lowerIndex])
          / (distCalArray[lowerIndex + 1] - distCalArray[lowerIndex])) * (distToSpeaker - distCalArray[lowerIndex]); // Linear
                                                                                                                     // interpolation
    }
  }

  // Whether the robot is in range to make a shot reliably.
  public boolean aimShotAvailable() {
    double maxShotDistance = 5.0; // The longest distance that the robot will make a shot from in meters.
    double speakerY = swerve.isBlueAlliance() ? 5.548 : Drivetrain.fieldWidth - 5.548; // The y-coordinate of the center
                                                                                       // of the speaker slot in meters,
                                                                                       // adjusted for alliance.
    double distToSpeaker = Math.sqrt(Math.pow(speakerY - swerve.getYPos(), 2) + Math.pow(swerve.getXPos(), 2)); // The
                                                                                                                // current
                                                                                                                // distance
                                                                                                                // to
                                                                                                                // the
                                                                                                                // speaker
                                                                                                                // based
                                                                                                                // on
                                                                                                                // the
                                                                                                                // robot's
                                                                                                                // position
                                                                                                                // on
                                                                                                                // the
                                                                                                                // field
                                                                                                                // in
                                                                                                                // meters.
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
  private boolean armCalibrate = false;
  private boolean climberReboot = false;
  private boolean climberCalibrate = false;
  private boolean climberLockoutToggle = false;

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
    SmartDashboard.putBoolean("Arm Recalibrate", armCalibrate);
    SmartDashboard.putBoolean("Climber Reboot", climberReboot);
    SmartDashboard.putBoolean("Climber Recalibrate", climberCalibrate);
    SmartDashboard.putBoolean("Climber Lockout Toggle", climberLockoutToggle);
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

    boolean currArmCalibrate = SmartDashboard.getBoolean("Arm Recalibrate", false);
    if (currArmCalibrate ^ armCalibrate) {
      arm.calibrate();
    }
    armCalibrate = currArmCalibrate;

    boolean currClimberReboot = SmartDashboard.getBoolean("Climber Reboot", false);
    if (currClimberReboot ^ climberReboot) {
      climber.reboot();
    }
    climberReboot = currClimberReboot;

    boolean currClimberCalibrate = SmartDashboard.getBoolean("Climber Recalibrate", false);
    if (currClimberCalibrate ^ climberCalibrate) {
      climber.calibrate();
    }
    climberCalibrate = currClimberCalibrate;

    boolean currClimberLockoutToggle = SmartDashboard.getBoolean("Climber Lockout Toggle", false);
    if (currClimberLockoutToggle ^ climberLockoutToggle) {
      climber.enableLockout();
    }
    climberLockoutToggle = currClimberLockoutToggle;
  }
}