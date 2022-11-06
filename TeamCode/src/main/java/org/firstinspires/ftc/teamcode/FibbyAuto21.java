package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.security.KeyStore;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;




@Autonomous(name="FibbyAuto21", group="Autonomous1")

public class FibbyAuto21 extends LinearOpMode {
    ColorSensor colorSensor;
    Telemetry.Item patternName;
    Telemetry.Item display;
    //    SampleRevBlinkinLedDriver.DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    BNO055IMU imu;
    private DcMotor LF_drive = null;
    private DcMotor RF_drive = null;
    private DcMotor LB_drive = null;
    private DcMotor RB_drive = null;
    private DcMotor Thrower = null;
    private DcMotor Intake = null;
    private DcMotor TS = null;
    private DcMotor Arm = null;
    private Servo Trigger = null;
    private Servo Grabber = null;
    private Servo Lift = null;
    // private CRServo Spin1 = null;
    private DistanceSensor distleft;
    private DistanceSensor distright;
    private DistanceSensor distfront;
    private DistanceSensor distback;
    DigitalChannel arm_limit;
    DigitalChannel arm_limit2;



    int RF_deg = 0;
    int RB_deg = 0;
    boolean Right = false;
    boolean forward = false;
    double Left_deg_double = 0;
    double LeftPower;
    double RightPower;
    double FrontPower;
    double BackPower;
    double heading = 0;
    double CorrectionFactor = 0.03;
    double BlueLineColor = 0.00;
    double RedLineColor = 1000;

    boolean Red_alliance = false;
    boolean FirstGyroRun = true;
    double rampmotorpower;
    boolean QuestionAnswered = false;
    boolean simplepark = false;
    //int NoRing = 0;
    //int SingleRing = 0;
    //int QuadRing = 2;
    int Rings =0;
    double dst_heading;
    Orientation angles;
    Acceleration gravity;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    private static final CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh\";\n";


    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;
    WebcamName webcamname = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;




    //-------------------------------------------------------------------------------------------------
    private void checkOrientation() {
// read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
// and save the heading
        heading = angles.firstAngle;
    }

    //-------------------------------------------------------------------------------------------------
    public void GyroDriveBase(boolean RampUp, int Course, double motorpower, boolean strafe, boolean right) {
        if (!Red_alliance)
            Course = Course * -1;
        if (FirstGyroRun) {
            FirstGyroRun = false;
            rampmotorpower = motorpower;
            //Set our starting motor powers before running ramp up
            if (rampmotorpower > 0 && RampUp)
                motorpower = 0.1;
            else if (rampmotorpower < 0 && RampUp)
                motorpower = -0.1;
        }
        checkOrientation();
        //Our gyro is upside down apparently, let's get it on it's feet!

        heading = heading * -1;
        //How far off course are we?? Store that in dst_heading
        dst_heading = (Course - heading);
        //Ramp up for motors
        if (motorpower < rampmotorpower && RampUp && rampmotorpower > 0)
            motorpower = motorpower + 0.03;
        else if (motorpower > rampmotorpower && RampUp && rampmotorpower < 0)
            motorpower = motorpower - 0.03;
        //Drive straight section
        if (!strafe) {
            //Set motor powers based on the direction we're moving and our heading
            //First half of if statement is for forward steering left, OR the last half is for reverse steering right
            if ((dst_heading < 0 && rampmotorpower > 0) || (dst_heading > 0 && rampmotorpower < 0)) {
                LeftPower = (motorpower - (dst_heading * CorrectionFactor * -1));
                RightPower = motorpower;
                //First half of if statement is for forward steering right , OR the last half is for reverse steering left
            } else if ((dst_heading > 0 && rampmotorpower > 0) || (dst_heading < 0 && rampmotorpower < 0)) {
                RightPower = (motorpower - (dst_heading * CorrectionFactor));
                LeftPower = motorpower;
            } else {
                LeftPower = motorpower;
                RightPower = motorpower;
            }
            LF_drive.setPower(-LeftPower);
            LB_drive.setPower(-LeftPower);
            RF_drive.setPower(RightPower);
            RB_drive.setPower(RightPower);
        } else if (strafe) {
            //Set motor powers based on the direction we're moving and our heading
            //First half of if statement is for strafing right and pivoting left, OR the last half is for reverse steering right
            if (dst_heading > 0 && !right) {
                FrontPower = (motorpower - (dst_heading * CorrectionFactor));
                BackPower = motorpower;
                //First half of if statement is for forward steering right , OR the last half is for reverse steering left
            } else if (dst_heading < 0 && !right) {
                BackPower = (motorpower - (dst_heading * CorrectionFactor * -1));
                FrontPower = motorpower;
            } else if (dst_heading < 0 && right) {
                FrontPower = (motorpower - (dst_heading * CorrectionFactor * -1));
                BackPower = motorpower;
                //First half of if statement is for forward steering right , OR the last half is for reverse steering left
            } else if (dst_heading > 0 && right) {
                BackPower = (motorpower - (dst_heading * CorrectionFactor));
                FrontPower = motorpower;
            } else {
                FrontPower = motorpower;
                BackPower = motorpower;
            }
            if (right) {
                LF_drive.setPower(-FrontPower);
                LB_drive.setPower(BackPower);
                RF_drive.setPower(-FrontPower);
                RB_drive.setPower(BackPower);
            } else {
                LF_drive.setPower(FrontPower);
                LB_drive.setPower(-BackPower);
                RF_drive.setPower(FrontPower);
                RB_drive.setPower(-BackPower);
            }
        }

    }

    //-------------------------------------------------------------------------------------------------
    //This function is used to consistently turn off powers to all of the motors and reset the encoders after every function.
    public void resetmotors() {
        RB_drive.setPower(0);
        LB_drive.setPower(0);
        RF_drive.setPower(0);
        LF_drive.setPower(0);
        //Thrower.setPower(0);
       // Intake.setPower(0);
        Arm.setPower(0);



        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //-------------------------------------------------------------------------------------------------
    public void drivedeg(int drivedegrees, double motorpower, int drivetime, boolean StopMotors, int Course, boolean RampUp) {
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FirstGyroRun = true;

        //The negative for the right motor to move in the correct direction

        //If we're drive backwards, we're looking at negative degrees and need to switch the operator in the while loop
        if (drivedegrees < 0) {
            while ((LF_drive.getCurrentPosition() <= -drivedegrees) && (LB_drive.getCurrentPosition() <= -drivedegrees) && (RB_drive.getCurrentPosition() >= drivedegrees) && (RF_drive.getCurrentPosition() >= drivedegrees) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        } else {
            while (((LF_drive.getCurrentPosition() >= -drivedegrees) && (LB_drive.getCurrentPosition() >= -drivedegrees) && (RB_drive.getCurrentPosition() <= drivedegrees) && (RF_drive.getCurrentPosition() <= drivedegrees)) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);

            }
        }
        telemetry.addData("Encoders RF RB LF LB", "%7d :%7d :%7d :%7d",
                RF_drive.getCurrentPosition(),
                RB_drive.getCurrentPosition(),
                LF_drive.getCurrentPosition(),
                LB_drive.getCurrentPosition());
        telemetry.update();


//turn off motors
        if (StopMotors)
            resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        sleep(drivetime);
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void drivedist(int distance, double motorpower, int drivetime, boolean FrontSensor, int Course, boolean RampUp) {
        if (FrontSensor) {
            // Check front distance sensor to be sure we're not already too close
            if (distfront.getDistance(DistanceUnit.INCH) > distance) {
                // Leave motors powered until we close the distance to less than we passed in
                while ((distfront.getDistance(DistanceUnit.INCH) > distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, motorpower, false, false);
                }
            } else {

                // Leave motors powered until we close the distance to less than we passed in
                while ((distfront.getDistance(DistanceUnit.INCH) < distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, -motorpower, false, false);
                }
            }


        }
        //else is for using back sensor because the boolean frontsensor is false.
        else {
            // Check back distance sensor to be sure we're not already too close
            if (distback.getDistance(DistanceUnit.INCH) > distance) {
                // Leave motors powered until we close the distance to less than we passed in
                while ((distback.getDistance(DistanceUnit.INCH) > distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, -motorpower, false, false);
                }
            } else {
                // Leave motors powered until we expand the distance to more than we passed in
                while ((distback.getDistance(DistanceUnit.INCH) < distance) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, motorpower, false, false);
                }
            }
        }
        //turn off motors
        resetmotors();
        //pause to give motors a rest NO GRINDING GEARS
        FirstGyroRun = true;
        sleep(drivetime);
    }

    //-------------------------------------------------------------------------------------------------
    public void SFColor(boolean Red, double motorpower, boolean Right, boolean StopOnLine, int Course, boolean RampUp) {
        if (Red) {
            //This will strafe to the red line
            while ((colorSensor.red() < RedLineColor) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        } else {
            //This will strafe to the blue line
            while ((colorSensor.blue() < BlueLineColor) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        }

//turn off motors
        if (StopOnLine)
            resetmotors();
        FirstGyroRun = true;
    }

    public void DriveColor(boolean Red, double motorpower, boolean StopOnLine, boolean RampUp, int Course) {
        if (Red) {
            //This will drive backwards to the Red Line
            while ((colorSensor.red() < RedLineColor) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        }
        // If Blue
        else {
            //This will drive to the Red Line
            while ((colorSensor.blue() < BlueLineColor) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, false, false);
            }
        }

//turn off motors if the boolean StopOnLine is true
        if (StopOnLine)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void SFdist(int dist, double motorpower, boolean distR, boolean StopMotors, boolean Right, int Course, boolean RampUp) {
        //If we want to use the LEFT distance sensor, then it will go through each of the following while loops to determine which direction to move.
        if (distR == false) {
            //This will strafe left using the left distance sensor moving TOWARD the object on the left
            if (!Right)
                while ((distleft.getDistance(DistanceUnit.INCH) >= dist) && (opModeIsActive())) {
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }
                //This will strafe right using the left distance sensor moving AWAY from the object on the left
            else while ((distleft.getDistance(DistanceUnit.INCH) <= dist) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }

        }
//Use the right Distance Sensor
        if (distR == true) {

            if (Right)
                while ((distright.getDistance(DistanceUnit.INCH) >= dist) && (opModeIsActive())) {
                    // Strafe Right
                    GyroDriveBase(RampUp, Course, motorpower, true, Right);
                }
            else while ((distright.getDistance(DistanceUnit.INCH) <= dist) && (opModeIsActive())) {
                //Strafe Left
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        }

//turn off motors
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    //Strafe using the motor encoders
    public void SF(int deg, double motorpower, int drivetime, boolean Right, boolean StopMotors, int Course, boolean RampUp) {
        LF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//Check to see if we're strafing right or left then run until encoder values are met then exit.
        if (Right)
            while (((LF_drive.getCurrentPosition() <= deg) && (RF_drive.getCurrentPosition() <= deg) && (LB_drive.getCurrentPosition() >= -deg) && (LF_drive.getCurrentPosition() >= -deg)) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }
        else
            while (((LB_drive.getCurrentPosition() <= deg) && (RB_drive.getCurrentPosition() <= deg) && (RF_drive.getCurrentPosition() >= -deg) && (LF_drive.getCurrentPosition() >= -deg)) && (opModeIsActive())) {
                GyroDriveBase(RampUp, Course, motorpower, true, Right);
            }


//turn off motors
        if (StopMotors)
            resetmotors();
        sleep(drivetime);
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void SpinGyro(double dest_heading, double motorpower, long drivetime, boolean Right, boolean ResetMotors) {
        if (Red_alliance == false) dest_heading = dest_heading * -1;
        long starttime = System.currentTimeMillis();
        checkOrientation();
        //If Right boolean is true it will spin right
        if (Right) {
            // Turn on the motors
            LF_drive.setPower(-motorpower);
            LB_drive.setPower(-motorpower);
            RF_drive.setPower(-motorpower);
            RB_drive.setPower(-motorpower);
            //Wait for heading to be achieved - if heading can't be achieved, timeout and exit the loop
            while ((heading >= dest_heading) && (System.currentTimeMillis() <= starttime + drivetime) && opModeIsActive()) {
                checkOrientation();
            }
        }
        //If false it will spin left
        else {
            // Turn on the motors
            LF_drive.setPower(motorpower);
            LB_drive.setPower(motorpower);
            RF_drive.setPower(motorpower);
            RB_drive.setPower(motorpower);
            while ((heading <= dest_heading) && (System.currentTimeMillis() <= starttime + drivetime) && (opModeIsActive())) {
                checkOrientation();
            }
        }

        //turn off motors if the boolean ResetMotors is true.
        if (ResetMotors)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void SFtime(int drivetime, double motorpower, boolean Right, boolean StopMotors, int Course, boolean RampUp) {
        long starttime = System.currentTimeMillis();
        //This will strafe left
        while ((System.currentTimeMillis() <= starttime + drivetime) && (opModeIsActive())) {
            GyroDriveBase(RampUp, Course, motorpower, true, Right);
        }
        //turn off motors
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    public void Drivetime(double motorpower, int drivetime, boolean StopMotors, int Course, boolean RampUp) {
        long starttime = System.currentTimeMillis();
        //Run Forwards
        while ((motorpower > 0) && (System.currentTimeMillis() <= starttime + drivetime) && (opModeIsActive())) {
            GyroDriveBase(RampUp, Course, motorpower, false, false);
        }
        //Run backwards
        while ((motorpower < 0) && (System.currentTimeMillis() <= starttime + drivetime) && (opModeIsActive())) {
            GyroDriveBase(RampUp, Course, motorpower, false, false);
        }
        //turn off motors if the boolean StopOnLine is true
        if (StopMotors)
            resetmotors();
        FirstGyroRun = true;
    }

    //-------------------------------------------------------------------------------------------------
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //-------------------------------------------------------------------------------------------------
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    //-------------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------
    //ArrayList<VuforiaTrackable> allTrackables, took this out of the varible list put back in for cam
    public void SimplePark (boolean Red_Alliance) {
        boolean RightTurn = true;
        boolean LeftTurn = false;
        boolean distR = true;
        boolean distL = false;
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
            distR = false;
            distL = true;
        }
        Lift.setPosition(0.555);
        // put auto program here
        telemetry.addData("running the thing 0", "Running the thing 0");
        telemetry.update();
        // drive out from wall to line
        drivedeg(1500, 0.3, 500, true, 0,true);
        // turn towards power shot 1
        SpinGyro(4.5,0.3,500,false,true);
        //shoot the cheerio
        Thrower.setPower(75);
        sleep(500);
        Trigger.setPosition(1);
        sleep(500);
        Trigger.setPosition(0.5);
        //Turn towards shot 2
        SpinGyro(11,0.3,500,false,true);
        //Throw the cheerio
        sleep(500);
        Trigger.setPosition(1);
        sleep(500);
        Trigger.setPosition(0.5);
        //Turn towards shot 3
        SpinGyro(16,0.3,500,false,true);
        // Throw the cheerio
        sleep(500);
        Trigger.setPosition(1);
        sleep(500);
        Trigger.setPosition(0.5);
        //turn off the thrower wheel
        Thrower.setPower(0);
        //drive to middle target square changed from 1800deg and course 35
        drivedeg(2000,0.3,500,true,30,true);
        //drop wobble
        while(arm_limit2.getState() == true) {
            Arm.setPower(0.6);
        }
        //sleep(1000); 
        Grabber.setPosition(0.94);
        sleep(700);
        // pull the arm back in
        while (arm_limit.getState() == true){
            Arm.setPower(-0.6);
        }
        Arm.setPower(0);
        // close the grabber back up
        Grabber.setPosition(0.92);
        //strafe to the left no stop
        SFtime(1000,0.4,false,false,35,true);
        //strafe to 0 no stop
        SFtime(3000,0.4,false,false,0,false);
        //park on line using distance
        drivedist(40,0.4,10000,true,0,false);
        // park on line
    }

    public void AAAAhhhh_Macarena (boolean Red_Alliance) {
        boolean RightTurn = true;
        boolean LeftTurn = false;
        boolean distR = true;
        boolean distL = false;
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
            distR = false;
            distL = true;
        }
        // put auto program here
        telemetry.addData("running the thing 1", "Running the thing 1");
        telemetry.update();

        // drive out from wall to line
        drivedeg(1200, 0.5, 500, true, 0,true);
        // lift the shooter
        Lift.setPosition(0.6);
        //strafe/ turn to top goal
        Thrower.setPower(75);
        SpinGyro(-1, 0.3, 1000, true, true);
        //fire awayyyyy
        //shoot the cheerio
        //1
        sleep(650);
        Trigger.setPosition(1);
        sleep(500);
        Trigger.setPosition(0.5);
        //2
        sleep(500);
        Trigger.setPosition(1);
        sleep(500);
        Trigger.setPosition(0.5);
        //3
        sleep(500);
        Trigger.setPosition(1);
        sleep(500);
        Trigger.setPosition(0.5);
        Thrower.setPower(0);
        //what ring!!!
        if (Rings == 0){
            //strafe to box 1
            SFdist(5, 0.5, true, true, true, 0, true);
            //drive forwards
            drivedeg(900, 0.3, 1000, true, 0,true);
            //drop wobble
            while(arm_limit2.getState() == true) {
                Arm.setPower(0.6);
            }
            //sleep(1000);
            Grabber.setPosition(0.94);
            sleep(700);
            // pull the arm back in
            while (arm_limit.getState() == true){
                Arm.setPower(-0.6);
            }
            Arm.setPower(0);
            // close the grabber back up
            Grabber.setPosition(0.92);
            //drive forwards
          drivedeg(100, 0.3, 1000, true, 0, true);
            //strafe back
            SFdist(30, 0.5,true,false, false,0,true);
            //park on line using distance
            drivedist(40,0.4,10000,true,0,false);

        }
        else if (Rings == 1){
            //drive to middle target square changed from 1800deg and course 30
            drivedeg(1300,0.4,500,true,25,true);
            //drop wobble
            while(arm_limit2.getState() == true) {
                Arm.setPower(0.6);
            }
            //sleep(1000);
            Grabber.setPosition(0.94);
            sleep(700);
            // pull the arm back in
            while (arm_limit.getState() == true){
                Arm.setPower(-0.6);
            }
            Arm.setPower(0);
            // close the grabber back up
            Grabber.setPosition(0.92);
            //strafe to the left no stop
            SFtime(1000,0.4,false,false,35,true);
            //strafe to 0 no stop 3000
            SFtime(2500,0.4,false,false,0,false);
            //park on line using distance
            drivedist(40,0.4,600,true,0,false);
            //new
            //strafe 30 dist  too wall
            SFdist(31,0.4,true,true,true,0,true);
            //intake on
            Intake.setPower(1);
            //back up to 40 too rings
            //drivedist(45,0.4,1000, false,0,true);
            Drivetime(-0.3,1300,true,0,true);
            sleep(100);
            // go to line
            drivedeg(190,0.4,1000,true,0,true);
            Intake.setPower(0);
            //shoot
            //strafe/ turn to top goal 11
            //start thrower

            Lift.setPosition(0.555);
            SpinGyro(13, 0.4, 10000, false, true);
            //fire awayyyyy
            //shoot the cheerio
            Thrower.setPower(75);
            sleep(700);
            Trigger.setPosition(1);
            sleep(400);
            Thrower.setPower(0);
            drivedeg(500,0.3,1000,true,0,true);
        }
        else{ //4 Rings
            //turn 90
            SpinGyro(-90,0.5,1000,false, true);
            //strafe to wall
            SFdist(7,0.6,true,false,true,-90, true);
            // back up to box
            drivedist(30,0.4,1000,false,-90,false);
            //drop wobble
            while(arm_limit2.getState() == true) {
                Arm.setPower(0.8);
            }
            //sleep(1000);
            Grabber.setPosition(0.94);
            sleep(700);
            // pull the arm back in
            while (arm_limit.getState() == true){
                Arm.setPower(-0.8);
            }
            Arm.setPower(0);
            // close the grabber back up
            Grabber.setPosition(0.92);
            //drive forwards
            drivedeg(450, 0.5, 900,true,-90, true);
            // strafe to line
            SFdist(45, 0.5, true,false, false,-90,true);
            //turn 90
           SpinGyro(0, 0.4,1000,true,true);
           //strafe 30 dist  too wall
            SFdist(31,0.4,true,true,true,0,true);
            //intake on
            Intake.setPower(1);
            //back up to 40 too rings
            //drivedist(45,0.4,1000, false,0,true);
            Drivetime(-0.3,2500,true,0,true);
            sleep(100);
            // go to line
            drivedeg(500,0.4,1000,true,0,true);
            Intake.setPower(0);
            //shoot
            //strafe/ turn to top goal
            SpinGyro(3, 0.4, 1000, false, true);
            //fire awayyyyy
            //shoot the cheerio
            Thrower.setPower(75);
            //1
            sleep(650);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0.5);
            //2
            sleep(500);
            Trigger.setPosition(1);
            sleep(500);
            Trigger.setPosition(0.5);
            //3
            sleep(500);
            Trigger.setPosition(1);
            Drivetime(0.3,850,true,0,false);
            sleep(500);
            Trigger.setPosition(0.5);
            Thrower.setPower(0);

        }

    }

    public void QuadRing (boolean Red_Alliance) {
        boolean RightTurn = true;
        boolean LeftTurn = false;
        boolean distR = true;
        boolean distL = false;
        if (!Red_alliance) {
            RightTurn = false;
            LeftTurn = true;
            distR = false;
            distL = true;
        }
        // put auto program here
        telemetry.addData("running the thing 4", "Running the thing 4");
        telemetry.update();
        //sleep(10000);
        //SFtime(1000,50,true,true,0,true);
    }
    //-------------------------------------------------------------------------------------------------
    public void runOpMode() {

        //Color sensor prep
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
            tfod.setClippingMargins(500,0,0,0);
        }
// gyro inputs from hub
        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro_parameters);





        //Mapping motor to variable names

        RB_drive = hardwareMap.dcMotor.get("RB_drive");
        LB_drive = hardwareMap.dcMotor.get("LB_drive");
        RF_drive = hardwareMap.dcMotor.get("RF_drive");
        LF_drive = hardwareMap.dcMotor.get("LF_drive");
        Thrower = hardwareMap.dcMotor.get("thrower");
        Intake = hardwareMap.dcMotor.get("intake");
        TS = hardwareMap.dcMotor.get("ts");
        Trigger = hardwareMap.servo.get("trigger");
        Grabber = hardwareMap.servo.get("grabber");
        // Spin1 = hardwareMap.crservo.get("spin1");
        Arm = hardwareMap.dcMotor.get("arm");
        Lift = hardwareMap.servo.get("lift");

        resetmotors();

        distleft = hardwareMap.get(DistanceSensor.class, "distleft");

        distright = hardwareMap.get(DistanceSensor.class, "distright");

        distfront = hardwareMap.get(DistanceSensor.class, "distfront");

        distback = hardwareMap.get(DistanceSensor.class, "distback");

        // place limit swiches here
        arm_limit = hardwareMap.get(DigitalChannel.class, "arm_limit");
        arm_limit.setMode(DigitalChannel.Mode.INPUT);

        arm_limit2 = hardwareMap.get(DigitalChannel.class, "arm_limit2");
        arm_limit2.setMode(DigitalChannel.Mode.INPUT);


        // Robot, squeeze into your box! the following code resets the robot to starting position
        // place bootup program here
        while (arm_limit.getState() == true){
            Arm.setPower(-0.5);
        }

        Arm.setPower(0);
        Grabber.setPosition(0.92);
//selecting auto
        Red_alliance = true;

        QuestionAnswered = false;
        /*telemetry.addLine("Y pos 1 - A for ? - X ? - B ?");
        telemetry.update();
        sleep(1000);
        while (QuestionAnswered == false) {
            if (gamepad1.y || gamepad2.y) {
                //Conquor!
                telemetry.addLine("Conquor the world!");
                simplepark = true;
                QuestionAnswered = true;
                telemetry.update();
            }
            if (QuestionAnswered = true);{
                telemetry.clear();
                telemetry.update();
            }

*/
        simplepark = true;
            while (!isStarted()) {
                //Will run during INIT
                    //Find the rings!
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            if (updatedRecognitions.size() == 0) Rings = 0;
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            //this is what is showing the numbers at the bottom of the screen
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                if (recognition.getLabel() == "Quad")
                                    Rings=4;
                                else if (recognition.getLabel() == "Single")
                                    Rings=1;
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                            }
                            telemetry.addData("Rings:", Rings);
                            telemetry.update();
                        }
                    }

            }

            if (tfod != null) {
                tfod.shutdown();
            }

             //AutoTransitioner.transitionOnStop(this, "FibbyTeleOp21");

            // what will run our auto when we have one
            //Start autonomous period -

            // if (LoopsRun) Loops(allTrackables, Red_alliance);



           // SimplePark(true);
          AAAAhhhh_Macarena (true);

            //turn off motors
            resetmotors();
        }

    }

