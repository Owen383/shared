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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

  private DcMotor frontLeftMotor;
  private DcMotor frontRightMotor;
  private DcMotor backLeftMotor;
  private DcMotor backRightMotor;

  @Override
  public void init() {

    frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
    backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

    frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    telemetry.addData("Status", "Initialized");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    frontLeftMotor.setPower(gamepad1.left_stick_y);
    frontRightMotor.setPower(gamepad1.left_stick_y);
    backLeftMotor.setPower(gamepad1.left_stick_y);
    backRightMotor.setPower(gamepad1.left_stick_y);
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
  }
}
