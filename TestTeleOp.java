/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "TestTeleOp")
//@Disabled
public class TestTeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  private DcMotor fl, fr, bl, br;

  double frPower = 0;
  double flPower = 0;
  double brPower = 0;
  double blPower = 0;

  @Override
  public void init() {

    fl = hardwareMap.get(DcMotor.class, "front_left_motor");
    fr = hardwareMap.get(DcMotor.class, "front_right_motor");
    bl = hardwareMap.get(DcMotor.class, "back_left_motor");
    br = hardwareMap.get(DcMotor.class, "back_right_motor");

    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    fl.setDirection(DcMotorSimple.Direction.FORWARD);
    bl.setDirection(DcMotorSimple.Direction.FORWARD);
    fr.setDirection(DcMotorSimple.Direction.REVERSE);
    br.setDirection(DcMotorSimple.Direction.REVERSE);

    telemetry.addData("Status", "Initialized");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {

  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();


  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());

    double drive = gamepad1.left_stick_y;
    double strafe = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;
    double precision = (gamepad1.left_trigger-1.5)/-1.5;
    double accelRate = 0.007;

    double flTargetPower = drive - strafe - turn;
    double frTargetPower = drive + strafe + turn;
    double blTargetPower = drive + strafe - turn;
    double brTargetPower = drive - strafe + turn;

    if(flPower < flTargetPower){
      flPower += accelRate;
    }else if(flPower > flTargetPower){
      flPower -= accelRate;
    }

    if(frPower < frTargetPower){
      frPower += accelRate;
    }else if(frPower > frTargetPower){
      frPower -= accelRate;
    }

    if(blPower < blTargetPower){
      blPower += accelRate;
    }else if(blPower > blTargetPower){
      blPower -= accelRate;
    }

    if(brPower < brTargetPower){
      brPower += accelRate;
    }else if(brPower > brTargetPower){
      brPower -= accelRate;
    }

    fl.setPower(flPower * precision);
    fr.setPower(frPower * precision);
    bl.setPower(blPower * precision);
    br.setPower(brPower * precision);
  }
}
