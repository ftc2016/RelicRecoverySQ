package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoConfiguration;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;


/**
 * TeleOp
 */
@TeleOp(name = "Driving Period")
public class RTeleOPREA extends OpMode {
    Servo jewel_vertical;
    DcMotor rightDrive;
    DcMotor leftDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    Servo servo_leftarmbottom;
    Servo servo_leftarmtop;
    Servo servo_rightarmbottom;
    Servo servo_rightarmtop;
    DcMotor RelicExtend;
    Servo turn_hand;
    Servo open_hand;
    DcMotor dcMotor_middle;
    float extend = 6;
    int relic_move_distance = 0;
    int relicExtend_minPosition;
    int relicExtend_maxPosition;
    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {
        dcMotor_middle = hardwareMap.dcMotor.get("dcMotor_middle");
        RelicExtend = hardwareMap.dcMotor.get("extend_hand");
        turn_hand = hardwareMap.servo.get("turn_hand");
        open_hand = hardwareMap.servo.get("open_hand");
        jewel_vertical = hardwareMap.servo.get("jewel_vertical");
        servo_leftarmbottom = hardwareMap.servo.get("left_glyph_top");
        servo_leftarmtop = hardwareMap.servo.get("left_glyph_bottom");
        servo_rightarmbottom = hardwareMap.servo.get("right_glyph_bottom");
        servo_rightarmtop = hardwareMap.servo.get("right_glyph_top");
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        // defining all the hardware
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
       // dcMotor_middle.setDirection(DcMotorSimple.Direction.REVERSE);
        //this puts the motors in reverse
    }


    @Override
    public void loop() {
        telemetry.addData("Current Relic Lift Postition:",turn_hand.getPosition());
        telemetry.update();

        float x1 = -gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        extend = 6;
        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;

        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;
        rightFrontPower += y1;
        rightBackPower += y1;

        // Handle strafing movement
        leftFrontPower += x1;
        leftBackPower -= x1;
        rightFrontPower -= x1;
        rightBackPower += x1;

        // Handle turning movement
        leftFrontPower -= x2;
        leftBackPower -= x2;
        rightFrontPower += x2;
        rightBackPower += x2;

        // Scale movement
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.5, .5);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.5, .5);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.5, .5);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.5, .5);
        }
        leftBackDrive.setPower(-leftBackPower);
        leftDrive.setPower(-leftFrontPower);
        rightDrive.setPower(-rightFrontPower);
        rightBackDrive.setPower(-rightBackPower);

        // Here you set the motors' power to their respected power double.
        // start of controller movements

        if (gamepad1.dpad_up)
            //moveForward(100);
            moveForwardOdometry(0.05,100);

        if (gamepad1.dpad_right)
            //moveRight(100);
            moveRightOdometry(0.05,100);

        if (gamepad1.dpad_down)
            //moveBackward(100);
            moveBackwardOdometry(0.05,100);

        if (gamepad1.dpad_left)
            //moveLeft(100);
            moveLeftOdometry(0.05,100);

        //press to raise linear lift
        if (gamepad2.dpad_up) {
            dcMotor_middle.setPower(-1);
        } else {
            dcMotor_middle.setPower(0);
        }
        //press to lower linear lift
        if (gamepad2.dpad_down) {
            dcMotor_middle.setPower(1);
        } else {
            dcMotor_middle.setPower(0);
        }

        if (gamepad2.a) {
            //in
            servo_rightarmbottom.setPosition(0.06);
            servo_rightarmtop.setPosition(0.43);
            servo_leftarmbottom.setPosition(0.64);
            servo_leftarmtop.setPosition(0.84);
        }
        if (gamepad2.b) {
            //out
            /*servo_rightarmbottom.setPosition(0.56);
            servo_rightarmtop.setPosition(0.81);
            servo_leftarmbottom.setPosition(0.3);
            servo_leftarmtop.setPosition(0.1);*/

            servo_rightarmbottom.setPosition(0.31);
            servo_rightarmtop.setPosition(0.69);
            servo_leftarmbottom.setPosition(0.39);
            servo_leftarmtop.setPosition(0.59);
        }

       //hold down to extend relic arm
  /*      double RelicScaleLeft = 0;
        RelicScaleLeft -= -gamepad2.left_stick_x;
        RelicScaleLeft = (float) Range.scale(RelicScaleLeft, -max, max, 0.3, .6);
        RelicExtend.setPosition(RelicScaleLeft);

        double RelicScaleRight = 0;
        RelicScaleRight -= gamepad2.left_stick_x;
        RelicScaleRight = (float) Range.scale(RelicScaleRight, -max, max,0.6,0.9);
        RelicExtend.setPosition(RelicScaleRight);
        */

        //if(gamepad2.dpad_left){
        //  RelicExtend.setPosition(.97);
        //else{
        // RelicExtend.setPosition(.46);

        if(gamepad2.left_trigger > .1){
            RelicExtend.setPower(-.45);
            telemetry.addData("current position", RelicExtend.getCurrentPosition());
            telemetry.update();
        }

        if(gamepad2.right_trigger > .1){
            RelicExtend.setPower(.1);
            telemetry.addData("current position", RelicExtend.getCurrentPosition());
            telemetry.update();
        }
        if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            RelicExtend.setPower(0);
        }

        if (jewel_vertical.getPosition() > 0.20)
            jewel_vertical.setPosition(0.03);

        //hold down to raise the relic arm
        if(gamepad2.y){
            turn_hand.setPosition( 0.67);
        }else{
            turn_hand.setPosition(0.15);
        }
        //hold down to grab relic
        if (gamepad2.right_bumper){
            open_hand.setPosition(0.25);
        }else{
            open_hand.setPosition(0.65);
        }
        telemetry.update();
    }





    public int distanceToCounts(double rotations1){
        int rotations = (int) Math.round (rotations1 * 100);
        return Math.round(rotations);
    }
    public void setPower(double power) {
        RelicExtend.setPower(power);
    }
    // end of loop

    public void moveForward(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        while (leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);
    }

    public void moveBackward(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        while (leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);
    }


    public void moveRight(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        while (leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);
    }


    public void moveLeft(double distance) {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        while (leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()) {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(100);
    }

    public void moveForwardOdometry(double power, int length) {
        setPower(power);
        sleep(length);
    }

    public void moveBackwardOdometry(double power, int length){
        setPower(power);
        sleep(length);
    }

    public void moveLeftOdometry(double power, int length){
        rightDrive.setPower(power);
        leftDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        leftBackDrive.setPower(power);
        sleep(length);
    }

    public void moveRightOdometry(double power, int length){
        rightDrive.setPower(-power);
        leftDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftBackDrive.setPower(-power);
        sleep(length);
    }



    @Override
    public void stop() {
    }
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.04, 0.08, 0.9, 0.11, 0.14, 0.17, 0.23, 0.29, 0.35, 0.42, 0.49, 0.59, 0.71, 0.84, 0.99, 1.00};
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}