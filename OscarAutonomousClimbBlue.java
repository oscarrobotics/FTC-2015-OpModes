package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by mike on 2/27/2016.
 */
public class OscarAutonomousClimbBlue extends  OscarAutonomousClimb {

  @Override
  public void init() {
    super.init();
    turnDegrees = -turnDegrees;
  }
}
