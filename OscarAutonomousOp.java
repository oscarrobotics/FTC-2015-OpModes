package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by George on 2/4/2016.
 */
public class OscarAutonomousOp extends OscarBaseOp {

  double CurrentDelay = 0.0;
  double DriveSpeed = 0.25;

  boolean ydown = false;
  boolean adown = false;
  boolean xdown = false;
  boolean bdown = false;

  @Override
  public void start() {
    super.start();
    resetStartTime();
  }

  private void ShowTelemetry() {
    telemetry.clearData();
    telemetry.addData("Y/A changes delay", "X/B changes drive speed");
    telemetry.addData("Delay: " + String.format("%.2f", CurrentDelay), "Speed: " + String.format("%.2f", DriveSpeed));
  }

  @Override
  public void init() {
    super.init();
    ShowTelemetry();
  }

  @Override
  public void init_loop() {
    super.init_loop();

    if (gamepad1 == null) {
      telemetry.addData("gamepad1", "null");
    } else {
      if (gamepad1.y) { // Want more delay
        if (!ydown) {
          CurrentDelay += 1.0;
        }
        ydown = true;
      } else ydown = false;

      if (gamepad1.a) { // Want less delay
        if (!adown) {
          CurrentDelay -= 1.0;
        }
        adown = true;
      } else adown = false;

      if (gamepad1.x) { // Want Slower drive
        if (!xdown) {
          DriveSpeed -= 0.05;
        }
        xdown = true;
      } else xdown = false;

      if (gamepad1.b) { // Want faster drive
        if (!bdown) {
          DriveSpeed += 0.05;
          ShowTelemetry();
        }
        bdown = true;
      } else bdown = false;

      DriveSpeed = Range.clip(DriveSpeed, 0.0, 1.0);

      ShowTelemetry();
    }

  }

  @Override
  public void loop() {
    super.loop();


    if (CurrentDelay > getRuntime()){
      telemetry.clearData();
      telemetry.addData("waiting", String.format("%.1f", CurrentDelay - getRuntime()));
    }

    if (getRuntime() >= (0.0 + CurrentDelay) && (getRuntime() <= 15.0 + CurrentDelay)) {
      motorHigh.setPower(0.5);
    } else {
      motorHigh.setPower(0.0);
    }

    if (getRuntime() >= (1.0 + CurrentDelay) && getRuntime() <= (6.0 + CurrentDelay)) {
      motorLow.setPower(0.5);
    } else {
      motorLow.setPower(0.0);
    }

    if (getRuntime() >= (2.0 + CurrentDelay) && getRuntime() <= (10.0 + CurrentDelay)) {
      motorLeft.setPower(DriveSpeed);
      motorRight.setPower(DriveSpeed);
    } else {
      motorLeft.setPower(0.0);
      motorRight.setPower(0.0);
    }
  }
}
