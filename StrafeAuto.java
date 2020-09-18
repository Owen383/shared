package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="StrafeAuto", group="")
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

        turn(90, 1.0);

        strafe(5200, 0, 135, 1.0, 0.0, 0.0);
        strafe(5200, 0, -45, 1.0, 0.0, 0.0);

        strafe(5200, 0, 0, 1.0, 0.0, 0.0);
        strafe(10400, 0, 180, 1.0, 0.0, 0.0);
        strafe(5200, 0, 0, 1.0, 0.0, 0.0);


        strafe(5200, 0, -90, 1.0, 0.0, 0.0);
        turn(100, 1.0);
        strafe(10400, 0, 90, 1.0, 0.0, 0.0);
        strafe(5200, 0, -90, 1.0, 0.0, 0.0);

        strafe(5200, 0, 112.5, 1.0, 0.0, 0.0);
        strafe(5200, 0, -67.2, 1.0, 0.0, 0.0);

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

    public void transition(double heading, double incomingAngle, double incomingPower, double outgoingAngle, double outgoingPower) {

        double accelRate = .00001;

        double drive = Math.cos((incomingAngle - imu.getAngle()) * (Math.PI / 180)) * incomingPower;
        double strafe = Math.sin((incomingAngle - imu.getAngle()) * (Math.PI / 180)) * incomingPower;

        double flPower = drive - strafe;
        double frPower = drive + strafe;
        double blPower = drive + strafe;
        double brPower = drive - strafe;

        double drive2 = Math.cos((outgoingAngle - imu.getAngle()) * (Math.PI / 180)) * outgoingPower;
        double strafe2 = Math.sin((outgoingAngle - imu.getAngle()) * (Math.PI / 180)) * outgoingPower;

        double flOut = drive2 - strafe2;
        double frOut = drive2 + strafe2;
        double blOut = drive2 + strafe2;
        double brOut = drive2 - strafe2;
        boolean bfl = false, bfr = false, bbl = false, bbr = false;

        while (!(bfl && bfr && bbl && bbr)) {

            bfl = flPower + .001 > flOut && flPower - .001 < flOut;
            bfr = frPower + .001 > frOut && frPower - .001 < frOut;
            bbl = blPower + .001 > blOut && blPower - .001 < blOut;
            bbr = brPower + .001 > brOut && brPower - .001 < brOut;

            if (flPower < flOut) {
                flPower += accelRate;
            } else if (flPower > flOut) {
                flPower -= accelRate;
            }

            if (frPower < frOut) {
                frPower += accelRate;
            } else if (frPower > frOut) {
                frPower -= accelRate;
            }

            if (blPower < blOut) {
                blPower += accelRate;
            } else if (blPower > blOut) {
                blPower -= accelRate;
            }

            if (brPower < brOut) {
                brPower += accelRate;
            } else if (brPower > brOut) {
                brPower -= accelRate;
            }

            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

            telemetry.addData("fl power: ", flPower);
            telemetry.addData("fr power: ", frPower);
            telemetry.addData("bl power: ", blPower);
            telemetry.addData("br power: ", brPower);
            telemetry.update();
    }
    }

    public void turn(double targetAngle, double targetPower){

        targetPower = Math.abs(targetPower);
        double accelRate = 0.01;
        boolean endLoop = false;
        double currentPower;
        double startAngle = imu.getAngle();

        if (imu.getAngle() > targetAngle) {
            targetPower *= -1;
        }

        while (!endLoop){
            double remainingAngle = Math.abs(targetAngle - imu.getAngle());
            double acceleratePower = Math.sqrt(accelRate * (Math.abs(imu.getAngle() - startAngle) + 1));
            double deceleratePower = Math.sqrt(accelRate * (remainingAngle));

            currentPower = Math.min(Math.min(acceleratePower, deceleratePower), targetPower);
            telemetry.addData("Current Power: ", currentPower);

            if(targetPower < 0){
                endLoop = imu.getAngle() < targetAngle;
            }else{
                endLoop = imu.getAngle() > targetAngle;
            }

            fl.setPower(currentPower * -1);
            fr.setPower(currentPower);
            bl.setPower(currentPower * -1);
            br.setPower(currentPower);
        }

    }

    public void strafe(double distance, double heading, double strafeAngle, double targetPower, double startPower, double endPower){

        distance = Math.abs(distance);
        startPower = Math.abs(startPower);
        targetPower = Math.abs(targetPower);
        endPower = Math.abs(endPower);

        resetMotors();
        double accelRate = 0.001;
        double currentPower;
        double currentDistance = 0;

        while (currentDistance < distance){

            double remainingDistance = distance - currentDistance;
            double acceleratePower = Math.sqrt(accelRate * (currentDistance + (1000 * Math.pow(startPower, 2)) + 1));
            double deceleratePower = Math.sqrt(accelRate * (remainingDistance + (1000 * Math.pow(endPower, 2))));

            currentPower = Math.min(Math.min(acceleratePower, deceleratePower), targetPower);
            telemetry.addData("Current Power: ", currentPower);


            double drive = Math.cos((strafeAngle - imu.getAngle()) * (Math.PI / 180)) * currentPower;
            double strafe = Math.sin((strafeAngle - imu.getAngle()) * (Math.PI / 180)) * currentPower;
            double turn = (heading - imu.getAngle()) * .000005 * Math.abs(currentPower);

            double measuredTicks = ((Math.abs(fr.getCurrentPosition()) + Math.abs(fl.getCurrentPosition()) + Math.abs(br.getCurrentPosition()) + Math.abs(bl.getCurrentPosition())) / 4.0);
            double angleAdjustment = (Math.abs(Math.abs(drive) - Math.abs(strafe))/currentPower -1) * -1;
            currentDistance = Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));


            fl.setPower(drive - strafe - turn);
            fr.setPower(drive + strafe + turn);
            bl.setPower(drive + strafe - turn);
            br.setPower(drive - strafe + turn);

            telemetry.addData("Drive: ", drive);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("IMU: ", imu.getAngle());
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
