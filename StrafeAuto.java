package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="StrafeAuto", group="Linear Opmode")
//@Disabled
public class StrafeAuto extends LinearOpMode {

    DcMotor fr, fl, br, bl;
    IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        initialize();
        waitForStart();

        strafe(5100, 0, 0, 0, 1.0, 0);
        strafe(5100, 0, 180, 0, 1.0, 0);
        strafe(5100, 0, 45, 0, 1.0, 0);
        strafe(5100, 0, -135, 0, 1.0, 0);
        setPowerAll(0.0);

    }

    public void initialize(){

        Utils.setHardwareMap(hardwareMap);

        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU("imu");

    }

    public void strafe(double distance, double heading, double strafeAngle, double targetPower, double startPower, double endPower){

        resetMotors();
        double drive;
        double strafe;
        double turn;
        double currentPower = startPower;
        double absDistance = Math.abs(distance);

        double measuredTicks;
        double diagonal;
        double currentDistance = 0;

        while (currentDistance < absDistance){

            drive = Math.cos((strafeAngle - imu.getAngle()) / 57.2958) * Math.abs(currentPower);
            strafe = Math.sin((strafeAngle - imu.getAngle()) / 57.2958) * Math.abs(currentPower);
            turn = (heading - imu.getAngle()) * .000005 * Math.abs(currentPower);

            measuredTicks = ((fr.getCurrentPosition() + fl.getCurrentPosition() + br.getCurrentPosition() + bl.getCurrentPosition()) / 4.0);
            diagonal = (Math.abs(Math.abs(drive) - Math.abs(strafe)) -1) * -1;
            currentDistance = Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * diagonal, 2));





            fl.setPower(drive - strafe - turn);
            fr.setPower(drive + strafe + turn);
            bl.setPower(drive + strafe - turn);
            br.setPower(drive - strafe + turn);

            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("IMU", imu.getAngle());
            telemetry.addData("Sin(45", Math.sin(45));
            telemetry.update();

        }

    }

    public void resetMotors(){
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPowerAll(double power){

        fr.setPower(power);
        fl.setPower(power);
        br.setPower(power);
        bl.setPower(power);

    }
}
