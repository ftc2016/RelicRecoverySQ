package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Blue1Auto_ENC")
public class Blue1Auto_ENC extends LinearOpMode {
    EncoderMethods blue1 = new EncoderMethods() {
        @Override
        public void runOpMode() throws InterruptedException {

        }
    };

    public double circumference_of_mecanum_wheels = 12.56;

    VuforiaLocalizer vuforia;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    //above we initialize all driving motors on our robot.
    private Servo servo_leftarmbottom;
    private Servo servo_leftarmtop;
    private Servo servo_rightarmbottom;
    private Servo servo_rightarmtop;
    // We use the servo's above for grabbing and controlling the glyph
    private Servo jewelVertical;
    private Servo jewelHorizontal;
    // We use jewelVertical and jewelHorizontal for knocking of the jewel at the autonomous beggining of the match
    private ColorSensor jewelColor;
    // this color sensor is used for sensing the color of the jewel
    DcMotor dcMotor_middle;
    // this is used for rasing and lowering the glyph holder

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dcMotor_middle = hardwareMap.dcMotor.get("dcMotor_middle");
        leftDrive = (DcMotor) hardwareMap.get("left_drive");
        rightDrive = (DcMotor) hardwareMap.get("right_drive");
        leftBackDrive = (DcMotor) hardwareMap.get("left_back_drive");
        rightBackDrive = (DcMotor) hardwareMap.get("right_back_drive");
        servo_leftarmbottom = hardwareMap.servo.get("left_glyph_bottom");
        servo_leftarmtop = hardwareMap.servo.get("left_glyph_top");
        servo_rightarmbottom = hardwareMap.servo.get("right_glyph_bottom");
        servo_rightarmtop = hardwareMap.servo.get("right_glyph_top");
        jewelVertical = hardwareMap.get(Servo.class, "jewel_vertical");
        jewelHorizontal = hardwareMap.get(Servo.class, "jewel_horizontal");
        jewelColor = (ColorSensor) hardwareMap.get("jewel_color");

        //List whether the motors are running using or without encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor_middle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // above we set up each motor to run with encoder
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //above we setup each motors direction of roatating
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // our license key is below replace it if needed
        parameters.vuforiaLicenseKey = "Ac0dfNr/////AAAAGVIj/WGQ20ausA1vrtvVr/MVsIByyuYLEuY0rlXewrVHaCzLe1iRW9q6+nvnKQOcZk7Sg2eOfib/cpA7NbtqD7E6tD2FegRNKdqTLlVwNE4oT2/Sv60VBMyMAEUSMk8ZTXMZ/4alBqwUqFe2ajodtauM+Vf2SGL1/GPcaeCvEDwK0J7mr2ggfyLcLKFcky3oZCrYOlRGKGKLbOkAFOUbJrMxrbjbcKCrP9vH4F3Sf2ArJIJnij+WzVk7NcLe25Sml0rppRjHvMscSjfHvK1U36G02f6SimOWPBu3zekvAuqJ+kG5Tl3WvlsLZLGzv8R35ovQYra9cYrZzhf7CdmGEo6HhDXaQdt3mWzWby7L30Nn";
        // if you do replace it please upload new .svg files to the developer.vuforia.com
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        RelicRecoveryVuMark vuMark;
         // above initializes vuforia
        //at this point everything is ready to go, the above happens when you press init and as soon as you press play everything starts

        waitForStart();
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //this only runs when "Init" button is pressed
            jewelColor.enableLed(true);
            sleep(1000);
            //set glyph grabber position
        servo_rightarmbottom.setPosition(0.9);
        servo_rightarmtop.setPosition(0.45);
        servo_leftarmbottom.setPosition(0.33);
        servo_leftarmtop.setPosition(0.35);
        // this grabs the glyph at the beggining of the match
        sleep(1000);
        // this sleep makes the program wait for 1 second
        dcMotor_middle.setPower(0.3);
        //the glyph raiser is given power rasing the glyph
        sleep(750);
        //sleeps for 250 miliseconds
        dcMotor_middle.setPower(0);

        sleep(1000);
        //start opMode loop
            while (opModeIsActive()) {
                relicTrackables.activate();
                //jewelColor.enableLed(true);
                jewelVertical.setDirection(Servo.Direction.FORWARD);
                jewelVertical.setPosition(0.68);
                sleep(3000);
                try{
                    if (jewelColor.red() > jewelColor.blue()) {
                        telemetry.addData("Blue Color:", jewelColor.blue());
                        telemetry.addData("Red Color:", jewelColor.red());
                        telemetry.update();
                        //knock of ball
                        jewelHorizontal.setPosition(0.1);
                        sleep(1000);
                        jewelHorizontal.setPosition(0.55);
                    } else {
                        telemetry.addData("Red Color:", jewelColor.red());
                        telemetry.addData("Blue Color:", jewelColor.blue());
                        telemetry.update();
                        //knock off ball
                        jewelHorizontal.setPosition(0.99);
                        sleep(1000);
                        jewelHorizontal.setPosition(0.55);
                    }
                }catch(Exception e){
                }
                sleep(1000);
                jewelVertical.setPosition(0);
                double power = 0.03;

                int leftCount = 0;
                int centerCount = 0;
                int rightCount = 0;
                for (int i = 0; i < 20; i++) {
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    sleep(100);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        if (vuMark == RelicRecoveryVuMark.LEFT) {
                            telemetry.addData("Key: ", "Left");
                            telemetry.update();
                            leftCount++;
                        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                            telemetry.addData("Key: ", "Middle");
                            telemetry.update();
                            centerCount++;
                        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                            telemetry.addData("Key: ", "Right");
                            telemetry.update();
                            rightCount++;
                        }
                    }
                    vuMark = null;
                }
                relicTrackables.deactivate();
                relicTemplate = null;
                parameters = null;
                this.vuforia = null;


                // more variable checking
                double distance = 2.8;

                if (leftCount > centerCount && leftCount > rightCount) {
                   //for left cryptobox
                    distance = distance - 0.68;
                    telemetry.addData("Position: ", "Left");
                    telemetry.update();

                } else if (centerCount > leftCount && centerCount > rightCount) {
                  //for center cryptobox
                    distance = distance;
                    telemetry.addData("Position: ", "Center");
                    telemetry.update();
                } else if (rightCount > centerCount && rightCount > leftCount) {
                   //for right cryptobox
                    distance = distance + 0.55;
                    telemetry.addData("Position: ", "Right");
                    telemetry.update();

                }

                sleep(1000);

               //Main Code Goes Here

                blue1.moveForward(distance);
                blue1.turnLeft(1.55);
                blue1.moveForward(1);
                //Open Arms
                servo_rightarmbottom.setPosition(0.42);
                servo_rightarmtop.setPosition(0);
                servo_leftarmbottom.setPosition(0.8);
                servo_leftarmtop.setPosition(0.8);
                blue1.moveBackward(0.45);

                //Stop opMode
                requestOpModeStop();
                break;
            }
        }

        //methods for encoder direction
    }

