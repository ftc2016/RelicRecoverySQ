package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;

/**
 * Created by user on 2/4/2018.
 */

public class EncoderMethods extends LinearOpMode{
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    public void moveForward(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts (distance);

        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() + (COUNTS)));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() + (COUNTS)));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() + (COUNTS)));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() + (COUNTS)));

        setPower(1);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }

    public void moveBackward(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts (distance);

        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() - (COUNTS)));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() - (COUNTS)));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() - (COUNTS)));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() - (COUNTS)));

        setPower(10);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }


    public void moveRight(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts (distance);

        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() - (COUNTS)));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() + (COUNTS)));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() + (COUNTS)));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() - (COUNTS)));

        setPower(1);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }


    public void moveLeft(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts (distance);

        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() + (COUNTS)));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition()- (COUNTS)));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() - (COUNTS)));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() + (COUNTS)));

        setPower(1);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }
    public void turnRight(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts(distance);

        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() - (COUNTS)));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() + (COUNTS)));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() - (COUNTS)));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() + (COUNTS)));

        setPower(1);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }
    public void turnLeft (double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts(distance);

        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() + (COUNTS)));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() - (COUNTS)));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() + (COUNTS)));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() - (COUNTS)));

        setPower(1);

        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }


    public void runToPosition() {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int distanceToCounts(double rotations1){
        int rotations = (int) Math.round (rotations1 * 1120);
        return Math.round(rotations);
    }

    public void setPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

