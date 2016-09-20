package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Team8214 on 2/7/2016.
 */
public class pautonomous extends LinearOpMode

{

  final double LOADING = .39;
  final double DUMPED = 1.0;
  final double CARRIED = 0;

  GyroSensor sensorGyro;
  OpticalDistanceSensor sensorODS;


  DcMotorController LightController;
  DcMotorController LMotorController;

  Servo Hopper;
  Servo AutonomousMan;


  DcMotor motorRight;
  DcMotor motorLeft;
  DcMotor Conveyor;
  DcMotor LightsLeft;
  DcMotor LightsRight;
  DcMotor Extension;
  DcMotor Winch1;
  DcMotor Winch2;

  ElapsedTime Time;


  double threshold = 0.80;
  //int encoderThreshold = 3;


  int encoder_cpr = 1120;
  int gear_ratio = 1;
  int xVal, yVal, zVal = 0;
  int heading = 0;

  double wheel_diameter = 2.56;
  double distance1 = 10.0;
  double distance2 = 6.0;
  double distance3 = 26.0;
  double distance4 = 36.0;

  double circumfrence = Math.PI * wheel_diameter;
  double rotations1 = distance1 / circumfrence;
  double rotations2 = distance2 / circumfrence;
  double rotations3 = distance3 / circumfrence;
  double rotations4 = distance4 / circumfrence;

  double COUNTS = encoder_cpr * rotations1 * gear_ratio;
  double COUNTS2 = encoder_cpr * rotations2 * gear_ratio;
  double COUNTS3 = encoder_cpr * rotations3 * gear_ratio;
  double COUNTS4 = encoder_cpr * rotations4 * gear_ratio;


  @Override
  public void runOpMode() throws InterruptedException {

    LightController = hardwareMap.dcMotorController.get("LightController");
    LMotorController = hardwareMap.dcMotorController.get("LMotorController");


    sensorODS = hardwareMap.opticalDistanceSensor.get("ODSSensor");
    sensorODS.enableLed(true);
    sensorGyro = hardwareMap.gyroSensor.get("gyroSensor");
    sensorGyro.calibrate();


    Conveyor = hardwareMap.dcMotor.get("Conveyor");
    LightsLeft = hardwareMap.dcMotor.get("LightsLeft");
    LightsRight = hardwareMap.dcMotor.get("LightsRight");
    Extension = hardwareMap.dcMotor.get("Extension");
    Winch1 = hardwareMap.dcMotor.get("Winch1");
    Winch2 = hardwareMap.dcMotor.get("Winch2");


    motorRight = hardwareMap.dcMotor.get("right_drive");
    motorLeft = hardwareMap.dcMotor.get("left_drive");
    motorLeft.setDirection(DcMotor.Direction.REVERSE);

    Hopper = hardwareMap.servo.get("Hopper");
    AutonomousMan = hardwareMap.servo.get("AutonomousMan");



    Hopper.setPosition(LOADING);
    AutonomousMan.setPosition(CARRIED);

    motorLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    motorRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

    waitForStart();

    int v_state = 0;

    sleep(200);
    Conveyor.setPower(1.0);

    while (opModeIsActive()) {
      double reflectance = sensorODS.getLightDetected();
      double header = sensorGyro.getHeading();

      telemetry.addData("Elapsed Time:", Time);
      telemetry.addData("Motor Target:", COUNTS);
      telemetry.addData("Motor Target:", COUNTS2);
      telemetry.addData("Motor Target:", COUNTS3);
      telemetry.addData("Motor Target:", COUNTS4);
      telemetry.addData("Left Position", motorLeft.getCurrentPosition());
      telemetry.addData("Right Position", motorRight.getCurrentPosition());
      telemetry.addData("Light Detected", sensorODS.getLightDetected());
      telemetry.addData("xVal:", String.format("%03d", xVal));
      telemetry.addData("yVal:", String.format("%03d", yVal));
      telemetry.addData("zVal:", String.format("%03d", zVal));
      telemetry.addData("Heading:", String.format("%03d", heading));

      switch (v_state) {

        case 0:

          motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
          motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

          v_state++;

          break;

        case 1:
          sleep(200);
          motorLeft.setTargetPosition((int) COUNTS);
          motorRight.setTargetPosition((int) COUNTS);
          motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          motorLeft.setPower(0.75);
          motorRight.setPower(0.75);
          v_state++;

          break;

        case 2:
          sleep(200);
          if (header == 315) {
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
          } else if ((header == 0) || (header > 315)) {
            motorRight.setPower(0.40);
            motorLeft.setPower(-0.40);
          } else {
            motorRight.setPower(0.40);
            motorLeft.setPower(-0.40);
          }

          v_state++;

          break;


        case 3:
          sleep(200);
          if (reflectance <= threshold) {
            motorRight.setPower(0.0);
            motorLeft.setPower(0.0);
          } else {
            motorLeft.setPower(0.55);
            motorRight.setPower(0.55);
          }


          v_state++;

          break;

        case 4:
          sleep(200);
          if (header == 270) {
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
          } else if (header > 270) {
            motorRight.setPower(0.40);
            motorLeft.setPower(-0.40);
          } else {
            motorRight.setPower(0.40);
            motorLeft.setPower(-0.40);
          }


          v_state++;

          break;

        case 5:
          sleep(200);
          motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
          motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

          v_state++;

          break;

        case 6:
          sleep(200);
          motorLeft.setTargetPosition((int) COUNTS2);
          motorRight.setTargetPosition((int) COUNTS2);
          motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          motorLeft.setPower(0.75);
          motorRight.setPower(0.75);

          v_state++;

          break;

        case 7:
          sleep(200);
          AutonomousMan.setPosition(DUMPED);

          v_state++;

          break;

        case 8:
          sleep(200);
          AutonomousMan.setPosition(CARRIED);

          v_state++;

          break;

        case 9:
          sleep(200);
          motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
          motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

          v_state++;

          break;

        case 10:
          sleep(200);
          motorLeft.setTargetPosition((int) COUNTS3);
          motorRight.setTargetPosition((int) COUNTS3);
          motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
          motorLeft.setPower(0.35);
          motorRight.setPower(0.35);

          v_state++;

          break;

      }
    }
  }
}