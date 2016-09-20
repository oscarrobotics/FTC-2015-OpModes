package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.ElapsedTime;

import newinventions.AdafruitIMU;

/**
 * Created by mike on 2/26/2016.
 */
public class OscarAutonomousClimb extends OscarBaseOp{

  private enum State // initializing states
  {
    STATE_INITIAL,
    STATE_DRIVING_FORWARD_FROM_START,
    STATE_REVERSE_FROM_PUSH,
    STATE_TURNING_TO_MOUNTAIN,
    STATE_CLIMB_MOUNTAIN,
    STATE_REACH_FOR_TILT,
    STATE_TILT_USING_ARMS_REACH,
    STATE_TILT_USING_ARMS_HOOK,
    STATE_TILT_USING_ARMS_PULL,
    STATE_STOP
  }

  public double turnDegrees = 91; // how many degrees relative to starting position are being turned.

  private double initialHeading = 0.0;
  private double targetHeading = 0.0;

  private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

  private State       mCurrentState;    // Current State Machine State.

  @Override
  public void init_loop() {
    super.init_loop();
  }

  @Override
  public void init() {
    super.init();
    long startTime = System.currentTimeMillis();
    try {
      boschBNO055 = new AdafruitIMU(hardwareMap, "bno055" // gyro

              //The following was required when the definition of the "I2cDevice" class was incomplete.
              //, "cdim", 5

              , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
              //addressing
              , (byte)AdafruitIMU.OPERATION_MODE_IMU);
    } catch (RobotCoreException e){
      telemetry.addData("Exception", e.getMessage());
    }

    telemetry.addData("1", "Gyro Init Time " + (System.currentTimeMillis()-startTime) + " ms.");


    setDrivePower(0, 0);        // Ensure motors are off
    resetDriveEncoders();
  }

  @Override
  public void start() {
    super.start();
    resetStartTime();
    initialHeading = yawAngle[1];
    newState(State.STATE_INITIAL);
    boschBNO055.startIMU();
  }

  @Override
  public void loop() {
    loopCounter++;
    // Send the current state info (state and time) back to first line of driver station telemetry.
    telemetry.addData("0", "Loop count " + loopCounter);

    super.loop();
    boschBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

    // Arms need to move independently of robot state so if they're not at the current target
    // they need to be always moving to the target
    if (moveArmsToEncoders){
      moveLowToPosition(lowEncoderTarget);
      moveHighToPosition(highEncoderTarget);
    }

    switch (mCurrentState) {
      case STATE_INITIAL:

        if (encodersAtZero())
        {
          rightEncoderTarget = 5000; // How far to drive forward initially. Can be ad
          leftEncoderTarget = 5000;

          // TODO: Determine proper arm positions to prepare for tilt move
          highEncoderTarget = -16000;

          moveArmsToEncoders = true;

          //TODO: Drive faster?
          runToPosition();
          motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + leftEncoderTarget);
          motorRight.setTargetPosition(motorRight.getCurrentPosition() + rightEncoderTarget);
          setDrivePower(0.4, 0.4);

          newState(State.STATE_DRIVING_FORWARD_FROM_START);
        }
        else
        {
          // Display Diagnostic data for this state.
          telemetry.addData("1 Initial ", String.format("L %5d - R %5d ", motorLeft.getCurrentPosition(),
                  motorRight.getCurrentPosition()));
        }


        break;

      case STATE_DRIVING_FORWARD_FROM_START:

          if (motorRight.getCurrentPosition() > rightEncoderTarget-5) { //we've driven far forward enough
            leftEncoderTarget = 3000;
            rightEncoderTarget = 3000;
            motorLeft.setTargetPosition(leftEncoderTarget);
            motorRight.setTargetPosition(rightEncoderTarget);
            setDrivePower(0.6, 0.6);
            newState(State.STATE_REVERSE_FROM_PUSH);
        }
        else
          {
            telemetry.addData("1", " Driving forward from start");
            telemetry.addData("2", "Current power L " + motorLeft.getPower() + " R " + motorRight.getPower());
          }
        break;

      case STATE_REVERSE_FROM_PUSH: // added in 3/5/2016
        if (motorRight.getCurrentPosition() < rightEncoderTarget+5) {
          // We've driven far enough back, need to start turning
          targetHeading = initialHeading + turnDegrees;
          useConstantPower();

          // TODO: Make sure this turns in the correct direction
          // TODO: Is turn speed OK?
          if (yawAngle[1] > targetHeading){
            setDrivePower(1, -1);
          } else {
            setDrivePower(-1, 1);
          }
          newState(State.STATE_TURNING_TO_MOUNTAIN);
        } else {
          // Display Diagnostic data for this state.

          telemetry.addData("1", String.format("Driving, R %5d:%5d ", rightEncoderTarget, motorRight.getCurrentPosition()));
          telemetry.addData("2", String.format("Driving, L %5d:%5d ", leftEncoderTarget, motorLeft.getCurrentPosition()));
          telemetry.addData("3", "Reverse from push");
          telemetry.addData("4", "Current power L " + motorLeft.getPower() + " R " + motorRight.getPower());
        }
        break;

      case STATE_TURNING_TO_MOUNTAIN:
        if (Math.abs(yawAngle[1] - targetHeading) < 1.0){
          // Turn complete
          useConstantSpeed();
          setDrivePower(0.5, 0.5);
          newState(State.STATE_CLIMB_MOUNTAIN);
        } else {
          // Still turning
          telemetry.addData("1", String.format("Turning; Current: %s Target: %s",  yawAngle[1], targetHeading));
          telemetry.addData("2", "Turning to mountain");
        }
        break;


      case STATE_CLIMB_MOUNTAIN:
        if (ultrasonic.getValue() >= 200) ultrasonicTargetCounter++; else ultrasonicTargetCounter = 0;

        if (ultrasonicTargetCounter >= 10) {
          setDrivePower(0, 0); // Stop climbing
          lowEncoderTarget = -16000;
          newState(State.STATE_REACH_FOR_TILT);
        } else {
          if (yawAngle[1] < targetHeading-5){
            setDrivePower(0.0, 0.5);
            telemetry.addData("1", "Climbing Left");
          }
          else if (yawAngle[1] > targetHeading+5){
            setDrivePower(0.5, 0.0);
            telemetry.addData("1", "Climbing Right");
          } else
          {
            setDrivePower(0.5, 0.5);
            telemetry.addData("1", "Climbing Straight");
          }
        }

        break;

      case STATE_REACH_FOR_TILT:
        if (Math.abs(lowEncoder() - lowEncoderTarget) < 10) {
          highEncoderTarget = -6500;
          newState(State.STATE_TILT_USING_ARMS_REACH);
        }
        break;

      case STATE_TILT_USING_ARMS_REACH:
        if (Math.abs(highEncoder() - highEncoderTarget) < 10) {
          lowEncoderTarget = -6000;
          newState(State.STATE_TILT_USING_ARMS_HOOK);
        }
        break;

      case STATE_TILT_USING_ARMS_HOOK:
        if (Math.abs(lowEncoder() - lowEncoderTarget) < 10) {
          newState(State.STATE_STOP);
        }
        break;

      case STATE_STOP:
        break;
    }
  }

  private void newState(State newState)
  {
    // Reset the state time, and then change to next state.
    mStateTime.reset();
    mCurrentState = newState;
  }

}
