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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This particular OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="FibbyTeleOp", group="TeleOp")
//@Disabled
public class FibbyTeleOp extends OpMode
{

    /* Declare OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear   = null;
    public DcMotor  rightRear  = null;
    public Servo grabber = null;

    public DcMotor lift = null;
    public DcMotor lift2 = null;
    DigitalChannel L_limit;
    DigitalChannel U_limit;
    private DcMotor parallelEncoder, perpendicularEncoder;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        grabber = hardwareMap.get(Servo.class,"Grabber");



        lift = hardwareMap.get(DcMotor.class,"Lift");
        lift2 = hardwareMap.get(DcMotor.class,"Lift2");

        L_limit = hardwareMap.get(DigitalChannel.class, "L_limit");
        U_limit = hardwareMap.get(DigitalChannel.class, "U_limit");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the digital channel to input.
        L_limit.setMode(DigitalChannel.Mode.INPUT);
        U_limit.setMode(DigitalChannel.Mode.INPUT);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

       grabber.setPosition(0);


        parallelEncoder = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);
      parallelEncoder.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;

        double liftPower;


        telemetry.addData("LF, RF, Parallel, Perpendicular", "Starting at %7d :%7d :%7d :%7d ",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                parallelEncoder.getCurrentPosition(),
                perpendicularEncoder.getCurrentPosition());
        telemetry.update();
        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)


        if (gamepad1.right_stick_y != 0) {
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = -gamepad1.right_stick_y;
            leftRearPower = -gamepad1.right_stick_y;
            rightRearPower = -gamepad1.right_stick_y;
        } else if (gamepad1.left_stick_x != 0) {
            leftFrontPower = gamepad1.left_stick_x;
            rightFrontPower = -gamepad1.left_stick_x;
            leftRearPower = gamepad1.left_stick_x;
            rightRearPower = -gamepad1.left_stick_x;
        } else if (gamepad1.left_trigger != 0) { //if left trigger is being pressed it spinning left
            leftFrontPower = -gamepad1.left_trigger;
            rightFrontPower = gamepad1.left_trigger;
            leftRearPower = gamepad1.left_trigger;
            rightRearPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger != 0) {
            leftFrontPower = gamepad1.right_trigger;
            rightFrontPower = -gamepad1.right_trigger;
            leftRearPower = -gamepad1.right_trigger;
            rightRearPower = gamepad1.right_trigger;
        } else {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftRearPower = 0;
            rightRearPower = 0;
        }
        //Limit switch - false == pressed, true == not pressed
        //This if statement is broken into two chunks, if the driver is trying to lower the lift AND the lower limit is NOT pressed,
        // OOOOORRRRRR if the driver is trying to raise the lift and the upper limit switch is not pressed then run the lift motor, otherwise don't.


        if ((gamepad2.right_stick_y > 0 && L_limit.getState() == true) || (gamepad2.right_stick_y < 0 && U_limit.getState() == true))
        //if (gamepad2.right_stick_y != 0)
        {
            if(gamepad2.right_stick_y < 0)
            liftPower = gamepad2.right_stick_y;
            else
            liftPower = gamepad2.right_stick_y * .75;
        } else {
            liftPower = 0;
        }

        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);

        leftRear.setPower(-leftRearPower);
        rightRear.setPower(-rightRearPower);

        lift.setPower(-liftPower);
        lift2.setPower(liftPower);

        //telemetry.addData("Servo Pos", grabber.getPosition());
        if (gamepad2.left_bumper){
            grabber.setPosition(0);
        }
        else if (gamepad2.right_bumper){
            grabber.setPosition(0.0765);
        }



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }
}
