package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import newinventions.AdafruitIMU;

/**
 * Created by George on 2/4/2016.
 */
public class OscarBaseOp extends OpMode {

  AdafruitIMU boschBNO055; // Initializing gyroscope object

  //The following arrays contain both the Euler angles reported by the IMU (indices = 0) AND the
  // Tait-Bryan angles calculated from the 4 components of the quaternion vector (indices = 1)
  volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

  final double ReleaseOpen = 0.95;
  final double ReleaseClosed = 0.05;
  final double LeftServoStart = 0.99;
  final double RightServoStart = 0.05;

  // Initializing motor objects
  DcMotor motorRight;
  DcMotor motorLeft;
  DcMotor motorLow;
  DcMotor motorHigh;

  // Initializing servo objects
  Servo servoRight;
  Servo servoLeft;
  Servo servoRelease;

  // Initializing ultrasonic, also declaring variables to detect when mountain opens up
  AnalogInput ultrasonic;
  int ultrasonicTargetCounter = 0;
  int loopCounter = 0;
  int ultrasonicCurrent = 0;
  int ultrasonicPrevious = 0;
  int changeInUltrasonic = 0;

  public boolean moveArmsToEncoders = false; // Don't move arms to a previous encoder setting

//  private int leftEncoderStart = 0;
//  private int rightEncoderStart = 0;
  private int highEncoderStart = 0; // Init high arm encoder to zero (wherever it starts = 0)
  private int lowEncoderStart = 0;  // Init low arm encoder to zero (wherever it starts = 0)

  public int leftEncoderTarget = 0; // Initialize left tread encoder target to zero
  public int rightEncoderTarget = 0; // Init right tread encoder target to zero
  public int highEncoderTarget = 0; // Init upper arm encoder target to zero
  public int lowEncoderTarget = 0; // Init lower arm encoder target to zero

//  public int leftDriveEncoder() {
//    return leftEncoderStart - motorLeft.getCurrentPosition();
//  }
//  public int rightDriveEncoder() {
//    return rightEncoderStart - motorRight.getCurrentPosition();
//  }

  // These two functions return values of where the encoder is on the arms at any given moment
  public int highEncoder(){
    return  highEncoderStart - motorHigh.getCurrentPosition();
  }
  public int lowEncoder(){
    return  lowEncoderStart - motorLow.getCurrentPosition();
  }


  @Override
  public void init() {

    /*
    In the robot controller app, similar to the old FTC system, you have to set up a map of all the different
    motor and servo objects. The strings indicate the exact name of the object in the application interface.
    For example, the motorLeft object declared earlier corresponds to "motor_left" hardware interface.
     */
    motorLeft = hardwareMap.dcMotor.get("motor_left");
    motorLeft.setDirection(DcMotor.Direction.FORWARD);
    motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    motorRight = hardwareMap.dcMotor.get("motor_right");
    motorRight.setDirection(DcMotor.Direction.REVERSE);
    motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    motorLow = hardwareMap.dcMotor.get("motor_low");
    motorLow.setDirection(DcMotor.Direction.REVERSE);
    motorLow.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    motorHigh = hardwareMap.dcMotor.get("motor_high");
    motorHigh.setDirection(DcMotor.Direction.REVERSE);
    motorHigh.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    servoLeft = hardwareMap.servo.get("servo_left");
    servoRight = hardwareMap.servo.get("servo_right");
    servoRelease = hardwareMap.servo.get("servo_release");
    servoLeft.setPosition(LeftServoStart);
    servoRight.setPosition(RightServoStart);
    servoRelease.setPosition(ReleaseClosed);

    ultrasonic = hardwareMap.analogInput.get("ultrasonic");

//    leftEncoderStart = motorLeft.getCurrentPosition();
//    rightEncoderStart = motorRight.getCurrentPosition();
    highEncoderStart = motorHigh.getCurrentPosition();
    lowEncoderStart = motorLow.getCurrentPosition();

    //telemetry.addData("2", String.format("Encoders L:R %d:%d", leftDriveEncoder(), rightDriveEncoder()));
  }

  @Override
  public void start() {
    super.start();
    long startTime = System.currentTimeMillis();
   //  boschBNO055.startIMU();//Set up the IMU as needed for a continual stream of I2C reads.
  }

  @Override
  public void loop() {
  }

  void setDrivePower(double leftPower, double rightPower) // Function that sets the drive power to whatever you set the variables as
  {
    // Clipping range, because controller doesn't take any values that aren't between -1 and 1.
    motorLeft.setPower(Range.clip(leftPower, -1, 1));
    motorRight.setPower(Range.clip(rightPower, -1, 1));
  }

  // Function that moves the upper arm to a given encoder target.
  void moveHighToPosition(int highEncoderTarget){
    if (moveArmsToEncoders){
      if (Math.abs(highEncoder() - highEncoderTarget) > 20){ // If difference between target and current pos is greater than 20
        // Need to move high motor
        int diff = highEncoder() - highEncoderTarget; // Difference between the two
        int sign = diff > 0 ?1  : -1; // Figure out the sign
        double level = 1.0;
        if (Math.abs(diff) < 1000) // Go slower when we're close
          level = 0.2;
        motorHigh.setPower(sign * level);
      }
      else
        motorHigh.setPower(0.0);
    }
  }

  void moveLowToPosition(int lowEncoderTarget){
    if (moveArmsToEncoders){
      if (Math.abs(lowEncoder() - lowEncoderTarget) > 20){
        // Need to move high motor
        int diff = lowEncoder() - lowEncoderTarget;
        int sign = diff > 0 ? 1 : -1;
        double level = 1.0;
        if (Math.abs(diff) < 1000) // Go slower when we're close
          level = 0.2;
        motorLow.setPower(sign * level);
      }
      else
        motorLow.setPower(0.0);
    }
  }

  public String encoderData() {
   return String.format("L %d/%d R %d/%d H %d/%d L %d/%d",
           motorLeft.getCurrentPosition(), leftEncoderTarget,
           motorRight.getCurrentPosition(), rightEncoderTarget,
            highEncoder(), highEncoderTarget,
            lowEncoder(), lowEncoderTarget);

  }
  public void setDriveMode(DcMotorController.RunMode mode)
  {
    // Ensure the motors are in the correct mode.
    if (motorLeft.getMode() != mode)
      motorLeft.setMode(mode);

    if (motorRight.getMode() != mode)
      motorRight.setMode(mode);
  }
  public void runToPosition()
  {
    setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
  }

  //--------------------------------------------------------------------------
  // useConstantSpeed ()
  // Set both drive motors to constant speed (requires encoders)
  //--------------------------------------------------------------------------
  public void useConstantSpeed()
  {
    setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
  }

  //--------------------------------------------------------------------------
  // useConstantPower ()
  // Set both drive motors to constant power (encoders NOT required)
  //--------------------------------------------------------------------------
  public void useConstantPower()
  {
    setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
  }

  void setEncoderTarget(int leftEncoder, int rightEncoder)
  {
    motorLeft.setTargetPosition(leftEncoderTarget = leftEncoder);
    motorRight.setTargetPosition(rightEncoderTarget = rightEncoder);
  }

  public void resetDriveEncoders()
  {
    setEncoderTarget(0, 0);
    setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
  }
  boolean encodersAtZero()
  {
    return ((Math.abs(motorLeft.getCurrentPosition()) < 5) && (Math.abs(motorRight.getCurrentPosition()) < 5));
  }
}
