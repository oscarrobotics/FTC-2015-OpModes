package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by George on 2/4/2016.
 */
public class OscarTeleOp extends OscarBaseOp {

  @Override
  public void loop() {
    super.loop();

    /// MLW - arm test remove

    super.loop();
    //boschBNO055.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

    if (gamepad1.y || gamepad1.start && !(gamepad1.a || gamepad2.b) || gamepad1.guide){
      moveArmsToEncoders = true;
      if (gamepad1.y){
        lowEncoderTarget = -5000;
        highEncoderTarget = -5000;
      }
      if (gamepad1.start || gamepad2.start){
        lowEncoderTarget = 0;
        highEncoderTarget = 0;
      }
      if (gamepad1.guide || gamepad2.start){
        // TODO: determine proper reach for hang
        lowEncoderTarget = -17000;
        highEncoderTarget = -38000;
      }
    } else
    {
      moveArmsToEncoders = false;
      lowEncoderTarget = lowEncoder();
      highEncoderTarget = highEncoder();
    }

    moveHighToPosition(highEncoderTarget);
    moveLowToPosition(lowEncoderTarget);

    /// MLW - arm test remove

    // TODO: Add climb on button

    // tank drive
    // note that if y equal -1 then joystick is pushed all of the way forward.
    float left = -gamepad1.left_stick_y;
    float right = -gamepad1.right_stick_y;

    // clip the right/left values so that the values never exceed +/- 1
    right = Range.clip(right, -1, 1);
    left = Range.clip(left, -1, 1);

    // scale the joystick value to make it easier to control
    // the robot more precisely at slower speeds.
    right = (float) scaleInput(right);
    left = (float) scaleInput(left);

    // write the values to the motors
    motorRight.setPower(right);
    motorLeft.setPower(left);

    // Lift control
    float lowlift = -gamepad2.left_stick_y;
    float highlift = -gamepad2.right_stick_y;

    // Climber release control
    if (gamepad2.a) {
      servoRelease.setPosition(ReleaseOpen);
    } else {
      servoRelease.setPosition(ReleaseClosed);
    }
    // Trigger servo control

    double leftTriggerPos = Math.max(Range.clip(gamepad1.left_trigger, 0, 0.9), Range.clip(gamepad2.left_trigger, 0.0, 0.8));
    double leftServoPos = Range.clip(1 - leftTriggerPos, 0, LeftServoStart);
    servoLeft.setPosition(leftServoPos);

    double rightTriggerPos = Math.max(Range.clip(gamepad1.right_trigger, 0, 0.9), Range.clip(gamepad2.right_trigger, 0.0, 0.8));
    double rightServoPos = Range.clip(rightTriggerPos, RightServoStart, 1);
    servoRight.setPosition(rightServoPos);

    if (!moveArmsToEncoders) {
      // Arm controls
      motorLow.setPower(lowlift);
      motorHigh.setPower(highlift);
    }

//    telemetry.addData("1", "A: " + gamepad1.toString());
//    telemetry.addData("2", "B: " + gamepad2.toString());
    telemetry.addData("3", "Enc. "+encoderData());
    telemetry.addData("Ultrasonic", ultrasonic.getValue());
    telemetry.addData("Heading ", yawAngle[1]);
  }

  double scaleInput(double dVal) {
    double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

    // get the corresponding index for the scaleInput array.
    int index = (int) (dVal * 16.0);

    // index should be positive.
    if (index < 0) {
      index = -index;
    }

    // index cannot exceed size of array minus 1.
    if (index > 16) {
      index = 16;
    }

    // get value from the array.
    double dScale = 0.0;
    if (dVal < 0) {
      dScale = -scaleArray[index];
    } else {
      dScale = scaleArray[index];
    }

    // return scaled value.
    return dScale;
  }
}
