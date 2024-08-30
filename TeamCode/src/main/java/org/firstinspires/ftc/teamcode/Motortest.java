package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Arrays;

@TeleOp(name="MotorTest", group="sai")
//@Disabled  This way it will run on the robot
public class Motortest extends OpMode{
    private final ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx

    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */

    //Touch Sensors
    //private DigitalChannel intakeSensor;

    //Motors
    private Rev2mDistanceSensor sideLeftDistanceSensor;
    private Rev2mDistanceSensor sideRightDistanceSensor;
    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx Viper;
    private DcMotorEx in;
    private DcMotorEx climb;


    private Servo flip;
    //private DcMotorEx Insertnamehere
    //private DcMotorEx Insertnamehere
    private Servo drone;
    private Servo claw;



    private double speedMod;
    private final boolean rumbleLevel = true;
    private double rotation = 0;
    final double TRIGGER_THRESHOLD  = 0.75;
    private int[] armLevelPosition = {0, 1000, 2000, 3000};
    private boolean isGrabbing = false;
    private int armLevel;
    private double previousRunTime;
    private double inputDelayInSeconds = .5;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors

        in = hardwareMap.get(DcMotorEx.class, "in");







        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete");


    }
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        previousRunTime = getRuntime();

    }
    //----------------------------------------------------------------------------------------------------------------------------------------------------
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//this will run the methods repeadtly

        intake();

//________________________________________________________________________________________________________________________________________________________________________________________________________________-
        telemetry.addData("Left Trigger Position", gamepad1.left_trigger);


        //Arm Slide Data

        // Show the elapsed game time and power for each wheel.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "wheelFL (%.2f), front right (%.2f), back left (%.2f),  right (%.2f)", wheelFL, wheelFR, wheelBL, wheelBR);

//        telemetry.addData("range", String.format("%.3f cm", sideDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range edited", sideDistanceSensor.getDistance(DistanceUnit.CM));

        telemetry.update();
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    public void intake() {
        // if (intakeSensor.getState()) {

        //gamepad2.rumble(1000);
        // }
        // else{
        if (gamepad2.right_bumper) { // shoots pixel
            in.setPower(1);
            // in.setPower(1);
        } else if (gamepad2.left_bumper) { // puts pixel into the bucket
            in.setPower(-1);
            //in.setPower(-1);
        } else {
            in.setPower(0);
            //   in.setPower(0);
        }
    }




    /*
     * Code to run ONCE after the driver hits STOP
     */

    /*
     * Code to run ONCE after the driver hits STOP
     */

}
//@Override
