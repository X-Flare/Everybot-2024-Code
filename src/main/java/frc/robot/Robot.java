// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Imports that allow the usage of REV Spark Max motor controllers
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;


public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kLaunchAndDrive = "launch drive";
  private static final String kLaunch = "launch";
  private static final String kDrive = "drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   *
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are uisng NEOs.
   * The rookie kit comes with CIMs which are brushed motors.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkBase leftRear = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkBase leftFront = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkBase rightRear = new CANSparkMax(3, MotorType.kBrushed);
  CANSparkBase rightFront = new CANSparkMax(4, MotorType.kBrushed);

  /*
   * A class provided to control your drivetrain. Different drive styles can be passed to differential drive:
   * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java
   */
  DifferentialDrive m_drivetrain;

  /*
   * Launcher motor controller instances.
   *
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (VictorSPX) to match your robot as necessary.
   *
   * Both of the motors used on the KitBot launcher are CIMs which are brushed motors
   */
  CANSparkBase m_launchWheel = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkBase m_feedWheel = new CANSparkMax(5, MotorType.kBrushless);

  RelativeEncoder m_launchWheelEncoder = m_launchWheel.getEncoder();

  CANSparkBase m_intakeWheel = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkBase m_intakeArm = new CANSparkMax(10, MotorType.kBrushless);

  CANSparkBase m_climberbox1 = new CANSparkMax(7, MotorType.kBrushless);  
  CANSparkBase m_climberbox2 = new CANSparkMax(8, MotorType.kBrushless);



  // SparkPIDController m_intakeArmPID = new SparkPIDController(m_intakeArm.getPIDController());
  RelativeEncoder m_intakeArmEncoder = m_intakeArm.getEncoder();
  RelativeEncoder m_climberBox1Encoder = m_climberbox1.getEncoder();
  RelativeEncoder m_climberBox2Encoder = m_climberbox2.getEncoder();
  DigitalInput m_intakeWheelLimitSwitch = new DigitalInput(0);

  // CANSparkBase m_testSpark = new CANSparkMax(62, MotorType.kBrushless);
  // RelativeEncoder m_tesRelativeEncoder = m_testSpark.getEncoder();

  /**
   * Roller Claw motor controller instance.
  */
  // CANSparkBase m_rollerClaw = new CANSparkMax(8, MotorType.kBrushed);
  /**
   * Climber motor controller instance. In the stock Everybot configuration a
   * NEO is used, replace with kBrushed if using a brushed motor.
   */
  // CANSparkBase m_climber = new CANSparkMax(7, MotorType.kBrushless);

    /**
   * The starter code uses the most generic joystick class.
   *
   * To determine which button on your controller corresponds to which number, open the FRC
   * driver station, go to the USB tab, plug in a controller and see which button lights up
   * when pressed down
   *
   * Buttons index from 0
   */
  Joystick m_driverController = new Joystick(0);


  Joystick m_manipController = new Joystick(1);


  // --------------- Magic numbers. Use these to adjust settings. ---------------

 /**
   * How many amps can an individual drivetrain motor use.
   */
  static final int DRIVE_CURRENT_LIMIT_A = 60;

  /**
   * How many amps the feeder motor can use.
   */
  static final int FEEDER_CURRENT_LIMIT_A = 60;

  /**
   * Percent output to run the feeder when expelling note
   */
  static final double FEEDER_OUT_SPEED = 1.0;

  /**
   * Percent output to run the feeder when intaking note
   */
  static final double FEEDER_IN_SPEED = -.2;

  /**
   * Percent output for amp or drop note, configure based on polycarb bend
   */
  static final double FEEDER_AMP_SPEED = 1.0;

  /**
   * How many amps the launcher motor can use.
   *
   * In our testing we favored the CIM over NEO, if using a NEO lower this to 60
   */
  static final int LAUNCHER_CURRENT_LIMIT_A = 60;

  /**
   * Percent output to run the launcher when intaking AND expelling note
   */
  static final double LAUNCHER_SPEED = 1.0;
  /**
   * RPM target for the launcher wheel
   */
  static final double LAUNCHER_RPM = 5000;

  /**
   * Percent output for scoring in amp or dropping note, configure based on polycarb bend
   * .14 works well with no bend from our testing
   */
  static final double LAUNCHER_AMP_SPEED = .17;
  /**
   * Percent output for the roller claw
   */
  static final double INTAKE_ARM_POWER = .15;
  /**
   * Percent output to help retain notes in the claw
   */
  static final double INTAKE_WHEEL_AMP = .65;
   
  static final double INTAKE_WHEEL_POWER = .5;
  /**
   * Percent output to power the climber
   */
  static final double CLIMBER_OUTPUT_POWER = .9;
  /**
   * Target encoder position for the intake arm while intaking
   */
  static final double INTAKE_ARM_INTAKE_POSITION = -7;
  /**
   * Target encoder position for the intake arm while scoring in the amp
   */
  static final double INTAKE_ARM_AMP_POSITION = -4.5;
  /**
   * Target encoder position for the intake arm while inside the robot
   */
  static final double INTAKE_ARM_INSIDE_POSITION = -3;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("launch note and drive", kLaunchAndDrive);
    m_chooser.addOption("launch", kLaunch);
    m_chooser.addOption("drive", kDrive);
    SmartDashboard.putData("Auto choices", m_chooser);



    /*
     * Apply the current limit to the drivetrain motors
     */
    leftRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    leftFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    rightRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);
    rightFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT_A);

    /*
     * Tells the rear wheels to follow the same commands as the front wheels
     */
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    /*
     * One side of the drivetrain must be inverted, as the motors are facing opposite directions
     */
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    /*
     * Launcher wheel(s) spinning the wrong direction? Change to true here.
     *
     * Add white tape to wheel to help determine spin direction.
     */
    m_feedWheel.setInverted(false);
    m_launchWheel.setInverted(false);

    /*
     * Apply the current limit to the launching mechanism
     */
    m_feedWheel.setSmartCurrentLimit(FEEDER_CURRENT_LIMIT_A);
    m_launchWheel.setSmartCurrentLimit(LAUNCHER_CURRENT_LIMIT_A);

    /*
     * Inverting and current limiting for roller claw and climber
     */
    // m_rollerClaw.setInverted(false);
    // m_climber.setInverted(false);

    // m_rollerClaw.setSmartCurrentLimit(60);
    // m_climber.setSmartCurrentLimit(60);

    /*
     * Motors can be set to idle in brake or coast mode.
     * 
     * Brake mode is best for these mechanisms
     */
    // m_rollerClaw.setIdleMode(IdleMode.kBrake);
    // m_climber.setIdleMode(IdleMode.kBrake);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test modes.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("encoder", m_intakeArmEncoder.getPosition());
    SmartDashboard.putNumber("rpm", m_launchWheelEncoder.getVelocity());
    SmartDashboard.putNumber("encoder climber", m_climberBox2Encoder.getPosition());

    // SmartDashboard.putNumber("Test Encoder", m_tesRelativeEncoder.getPosition());

  }

  /*
   * Auto constants, change values below in autonomousInit()for different autonomous behaviour
   *
   * A delayed action starts X seconds into the autonomous period
   *
   * A time action will perform an action for X amount of seconds
   *
   * Speeds can be changed as desired and will be set to 0 when
   * performing an auto that does not require the system
   */
  double AUTO_LAUNCH_DELAY_S;
  double AUTO_DRIVE_DELAY_S;

  double AUTO_DRIVE_TIME_S;

  double AUTO_DRIVE_SPEED;
  double AUTO_LAUNCHER_SPEED;

  double autonomousStartTime;

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    leftRear.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);

    AUTO_LAUNCH_DELAY_S = 2;
    AUTO_DRIVE_DELAY_S = 4; //changed 3/1/2024 from 3

    AUTO_DRIVE_TIME_S = 3.0; //changed 3/1/2024 from 2.0
    AUTO_DRIVE_SPEED = -0.5;
    AUTO_LAUNCHER_SPEED = 1;
    
    /*
     * Depeding on which auton is selected, speeds for the unwanted subsystems are set to 0
     * if they are not used for the selected auton
     *
     * For kDrive you can also change the kAutoDriveBackDelay
     */
    if(m_autoSelected == kLaunch)
    {
      AUTO_DRIVE_SPEED = 0;
    }
    else if(m_autoSelected == kDrive)
    {
      AUTO_LAUNCHER_SPEED = 0;
    }
    else if(m_autoSelected == kNothingAuto)
    {
      AUTO_DRIVE_SPEED = 0;
      AUTO_LAUNCHER_SPEED = 0;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    /*
     * Spins up launcher wheel until time spent in auto is greater than AUTO_LAUNCH_DELAY_S
     *
     * Feeds note to launcher until time is greater than AUTO_DRIVE_DELAY_S
     *
     * Drives until time is greater than AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S
     *
     * Does not move when time is greater than AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S
     */
    if(timeElapsed < AUTO_LAUNCH_DELAY_S)
    {
      m_launchWheel.set(AUTO_LAUNCHER_SPEED);
      m_feedWheel.set(AUTO_LAUNCHER_SPEED);
      m_drivetrain.arcadeDrive(0, 0);

    }
    else if(timeElapsed < AUTO_DRIVE_DELAY_S)
    {
      m_intakeWheel.set(-INTAKE_WHEEL_POWER);
      m_drivetrain.arcadeDrive(0, 0);
    }
    else if(timeElapsed < AUTO_DRIVE_DELAY_S + AUTO_DRIVE_TIME_S)
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
      m_intakeWheel.set(0);
      m_drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, 0);
    }
    else
    {
      m_drivetrain.arcadeDrive(0, 0);
    }
    /* For an explanation on differintial drive, squaredInputs, arcade drive and tank drive see the bottom of this file */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    /*
     * Motors can be set to idle in brake or coast mode.
     *
     * Brake mode effectively shorts the leads of the motor when not running, making it more
     * difficult to turn when not running.
     *
     * Coast doesn't apply any brake and allows the motor to spin down naturally with the robot's momentum.
     *
     * (touch the leads of a motor together and then spin the shaft with your fingers to feel the difference)
     *
     * This setting is driver preference. Try setting the idle modes below to kBrake to see the difference.
     */
    leftRear.setIdleMode(IdleMode.kCoast);
    leftFront.setIdleMode(IdleMode.kCoast);
    rightRear.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /*
     * Spins up the launcher wheel
     */
    if (m_manipController.getRawButton(1)) {
      m_launchWheel.set(LAUNCHER_SPEED);
      m_feedWheel.set(LAUNCHER_SPEED);
    }
    else if(m_manipController.getRawButtonReleased(1))
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
    }

    /*
     * Spins feeder wheel, wait for launch wheel to spin up to full speed for best results
     */
    // TODO: add logic to wait for the launcher wheel to spin up before feeding
    // TODO: add logic to feed the note from the intake into the feeder wheel
    if (m_manipController.getRawButton(6))
    {
      // if(m_launchWheelEncoder.getVelocity() > LAUNCHER_RPM) // Starter code for waiting for the launcher wheel to spin up
      // {
        m_intakeWheel.set(-INTAKE_WHEEL_POWER); // Starter code for feeding the note into the feeder wheel
      // }
    }
    else if(m_manipController.getRawButtonReleased(6))
    {
      m_intakeWheel.set(0);
    }

    /*
     * While the button is being held spin both motors to intake note
     */
    //TODO Remove or modify
    if(m_manipController.getRawButton(5))
    {
      m_launchWheel.set(-LAUNCHER_SPEED);
      m_feedWheel.set(FEEDER_IN_SPEED);
      m_intakeWheel.set(INTAKE_WHEEL_POWER);
    }
    else if(m_manipController.getRawButtonReleased(5))
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
      m_intakeWheel.set(0);
    }

    /*
     * While the amp button is being held, spin both motors to "spit" the note
     * out at a lower speed into the amp
     *
     * (this may take some driver practice to get working reliably)
     */
    //TODO Remove or modify
    if(m_driverController.getRawButton(2))
    {
      m_feedWheel.set(LAUNCHER_AMP_SPEED);
      m_launchWheel.set(LAUNCHER_AMP_SPEED);
      m_intakeWheel.set(LAUNCHER_AMP_SPEED+.1);
    }
    else if(m_manipController.getRawButtonReleased(2))
    {
      m_feedWheel.set(0);
      m_launchWheel.set(0);
      m_intakeWheel.set(0);
    }

    //TODO: Add functionality so that when the button is pressed to run the intake,
    //the intake arm will lower to a certain position. When the button is released, the
    //intake arm will raise to a certain position.
    //Also add a button in this same logic so that the arm will be raised/lowered to a certain position
    //so that the robot can score in the Amp.

    /**
     * Starter code for the above functionality
     * Uncomment when ready to use
     * You will need to modify the encoder positions and power levels to match your robot
     * 
     * BUTTON 3 is X (on xbox)
     * BUTTON 4 is Y (on xbox)
     * 
     * Hold down button 3 to intake note. When the button is held, the intake arm will lower to a certain position.
     * When the button is released, the intake arm will raise to a certain position.
     * 
     * Hold down button 4 to expel note. When the button is held, the intake arm will lower to a certain position.
     * When the button is released, the intake arm will raise to a certain position.
     * 
     * The intake arm will also raise to a certain position when the button is released.
     */
    if(m_manipController.getRawButton(3))
    {

      // This code will lower the intake arm to a certain position when the button is held
      if(m_intakeArmEncoder.getPosition() > INTAKE_ARM_INTAKE_POSITION)
      {
        m_intakeArm.set(-INTAKE_ARM_POWER); // This will lower the intake arm
      } else {
        m_intakeArm.set(0); // This will stop the intake arm from moving
      }

      // This will stop the intake wheel from spinning if the limit switch is pressed
      // if(m_intakeWheelLimitSwitch.get()) 
      // {
      //   m_intakeWheel.set(0); // This will stop the intake wheel from spinning
      // } else {
        m_intakeWheel.set(INTAKE_WHEEL_AMP); // This will spin the intake wheel
      // }

    }
    else if(m_manipController.getRawButton(4)) // This code will raise the intake arm to a certain position when the button is held
    {

      // If the arm is already at Amp height, stop the arm from moving and spin out the intake wheel
      if (m_intakeArmEncoder.getPosition() > INTAKE_ARM_AMP_POSITION && m_intakeArmEncoder.getPosition() < INTAKE_ARM_AMP_POSITION-0.25)
      {
        m_intakeArm.set(0);
      }
      else if(m_intakeArmEncoder.getPosition() < INTAKE_ARM_AMP_POSITION-.25) // If the arm is below the Amp height, raise the arm
      {
        m_intakeArm.set(0.03);
      } 
      else if(m_intakeArmEncoder.getPosition() > INTAKE_ARM_AMP_POSITION) // If the arm is above the Amp height, lower the arm
      {
        if(m_intakeArmEncoder.getPosition() > INTAKE_ARM_AMP_POSITION+1)
        {
          m_intakeArm.set(-INTAKE_ARM_POWER);
        } else if (m_intakeArmEncoder.getPosition() > INTAKE_ARM_AMP_POSITION+0.15){
          m_intakeArm.set(-0.03);
        } else {
          m_intakeArm.set(-0.01);
        }
      }
      
    }
    else // This code will raise the intake arm to a certain position when the button is released
    {

      // If the arm is not inside the robot, raise the arm
      if(m_intakeArmEncoder.getPosition() < INTAKE_ARM_INSIDE_POSITION)
      {
        m_intakeArm.set(INTAKE_ARM_POWER);
      } 
      else // If the arm is inside the robot, stop the arm from moving
      {
        m_intakeArm.set(0);
      }
      // This will stop the intake wheel from spinning if no button is pressed

    }
    if (m_manipController.getRawButtonReleased(3)||m_manipController.getRawButtonReleased(4)){
      m_intakeWheel.set(0);
    }

    if(m_manipController.getRawAxis(3) > 0.05) {
      m_intakeWheel.set(-INTAKE_WHEEL_POWER);
    } else if (!m_manipController.getRawButton(3) && !m_manipController.getRawButton(6) && !m_manipController.getRawButton(5)) {
      m_intakeWheel.set(0);
    }

    //climber code

    if(m_manipController.getRawAxis(1) > .2) {
      m_climberbox1.set(CLIMBER_OUTPUT_POWER);
    } else if (m_manipController.getRawAxis(1) < -.2 && m_climberBox1Encoder.getPosition() < -.3) {
      m_climberbox1.set(-CLIMBER_OUTPUT_POWER);
    } else if((m_manipController.getRawAxis(1) < .21 && m_manipController.getRawAxis(1) > -.21)||m_climberBox1Encoder.getPosition() > -.3) {
      m_climberbox1.set(0);
    }

    if(m_manipController.getRawAxis(5) > .2) {
      m_climberbox2.set(CLIMBER_OUTPUT_POWER);
    } else if (m_manipController.getRawAxis(5) < -.2 && m_climberBox2Encoder.getPosition() < -.3) {
      m_climberbox2.set(-CLIMBER_OUTPUT_POWER);
    } else if((m_manipController.getRawAxis(5) < .21 && m_manipController.getRawAxis(5) > -.21)||m_climberBox2Encoder.getPosition() > -.3) {
      m_climberbox2.set(0);
    }
    

    /**
     * BUTTON 3 is X (on xbox)
     * BUTTON 4 is Y (on xbox)
     * 
     * Hold down button 3 to intake note
     * Hold down button 4 to expel note
     */
    // if(m_manipController.getRawButton(9))//BUTTON 3 IS X (on xbox)
    // {
    //   m_intakeWheel.set(INTAKE_WHEEL_POWER);
    // }
    // else if(m_manipController.getRawButton(10))//BUTTON 4 IS Y (on xbox)
    // {
    //   m_intakeWheel.set(-INTAKE_WHEEL_POWER);
    // }
    // else
    // {
    //   m_intakeWheel.set(1);
    // }

    /**
     * POV is the D-PAD (directional pad) on your controller, 0 == UP and 180 == DOWN
     * 
     * Hold down d-pad up to raise the intake arm
     * Hold down d-pad down to lower the intake arm
     */
    // if(m_manipController.getPOV() == 0)//get pov (0) IS d-pad up (on xbox)
    // {
    //   m_intakeArm.set(INTAKE_ARM_POWER);
    // }
    // else if(m_manipController.getPOV() == 180)//get pov (180) IS d-pad down (on xbox)
    // {
    //   m_intakeArm.set(-INTAKE_ARM_POWER);
    // }
    // else
    // {
    //   m_intakeArm.set(0);
    // }

    /**
     * POV is the D-PAD (directional pad) on your controller, 0 == UP and 180 == DOWN
     * 
     * After a match re-enable your robot and unspool the climb
     */
    //TODO Remove or modify
    // if(m_manipController.getPOV() == 0)
    // {
    //   m_climber.set(1);
    // }
    // else if(m_manipController.getPOV() == 180)
    // {
    //   m_climber.set(-1);
    // }
    // else
    // {
    //   m_climber.set(0);
    // }
  
    /*
     * Negative signs are here because the values from the analog sticks are backwards
     * from what we want. Pushing the stick forward returns a negative when we want a
     * positive value sent to the wheels.
     *
     * If you want to change the joystick axis used, open the driver station, go to the
     * USB tab, and push the sticks determine their axis numbers
     *
     * This was setup with a logitech controller, note there is a switch on the back of the
     * controller that changes how it functions
     */
    m_drivetrain.arcadeDrive(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(4), false);
  }
}

/*
 * The kit of parts drivetrain is known as differential drive, tank drive or skid-steer drive.
 *
 * There are two common ways to control this drivetrain: Arcade and Tank
 *
 * Arcade allows one stick to be pressed forward/backwards to power both sides of the drivetrain to move straight forwards/backwards.
 * A second stick (or the second axis of the same stick) can be pushed left/right to turn the robot in place.
 * When one stick is pushed forward and the other is pushed to the side, the robot will power the drivetrain
 * such that it both moves fowards and turns, turning in an arch.
 *
 * Tank drive allows a single stick to control of a single side of the robot.
 * Push the left stick forward to power the left side of the drive train, causing the robot to spin around to the right.
 * Push the right stick to power the motors on the right side.
 * Push both at equal distances to drive forwards/backwards and use at different speeds to turn in different arcs.
 * Push both sticks in opposite directions to spin in place.
 *
 * arcardeDrive can be replaced with tankDrive like so:
 *
 * m_drivetrain.tankDrive(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(5))
 *
 * Inputs can be squared which decreases the sensitivity of small drive inputs.
 *
 * It literally just takes (your inputs * your inputs), so a 50% (0.5) input from the controller becomes (0.5 * 0.5) -> 0.25
 *
 * This is an option that can be passed into arcade or tank drive:
 * arcadeDrive(double xSpeed, double zRotation, boolean squareInputs)
 *
 *
 * For more information see:
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
 *
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java
 *
 */
