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
// r

package org.firstinspires.ftc.teamcode.drive.stinger;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// we are the rizzly bears and we are sigma

@TeleOp(name="EveryBot", group="Iterative OpMode")
@Disabled
public class EveryBot extends OpMode
{
    // Declare OpMode members./.c
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftLift = null;


    private DcMotor rightLift = null;

    private Servo wrist = null;

    private Servo claw = null;

    final static double clawStart = 0.1;

    public static double clawMin = 0.4;

    public static double clawMax = 0.8;

    public static double clawSpeed = 0.01;

    public double clawPosition = 0.5;

    public static double wristMin = 0.4;

    public static double wristMax = 1.0; 

    public static double wristSpeed = 0.01;

    public double wristPosition = 0.5;

    public double driveSpeed = 1.0;

    public double liftSpeed = 0.5;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");  //Control Hub 1
        rightBack = hardwareMap.get(DcMotor.class,"rightBack"); //Control Hub 0
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");  //Expansion Hub 1
        rightFront = hardwareMap.get(DcMotor.class,"rightFront"); //Expansion Hub 0
        leftLift   = hardwareMap.get(DcMotor.class,"leftLift");   //Control Hub 3
        rightLift  = hardwareMap.get(DcMotor.class,"rightLift");  //Control Hub 2
        wrist = hardwareMap.get(Servo.class, "wrist");          //Control Hub Servo 1
        claw = hardwareMap.get(Servo.class, "claw");              //Control Hub Servo 0
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;
        double leftLiftPower;
        double rightLiftPower;
        double wristPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0) ;
        rightBackPower   = Range.clip(drive - turn + strafe, -1.0, 1.0) ;
        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0) ;
        rightFrontPower   = Range.clip(drive - turn - strafe, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        //leftBackPower  = -gamepad1.left_stick_y ;
        //rightBackPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftBack.setPower(leftBackPower * driveSpeed);
        rightBack.setPower(rightBackPower * driveSpeed);
        leftFront.setPower(leftFrontPower * driveSpeed);
        rightFront.setPower(rightFrontPower * driveSpeed);

        double lift = -gamepad2.left_stick_y;
        leftLiftPower = Range.clip(lift, -1.0, 1.0);
        rightLiftPower = Range.clip(lift, -1.0, 1.0);

        leftLift.setPower(leftLiftPower * liftSpeed);
        rightLift.setPower(rightLiftPower * liftSpeed);



        if (gamepad2.right_stick_y > 0 && wristPosition < wristMax)
            wristPosition += wristSpeed;
        if (gamepad2.right_stick_y < 0 && wristPosition >= wristMin)
            wristPosition -= wristSpeed;


        if (gamepad2.right_trigger > 0 && clawPosition < clawMax)
            clawPosition += clawSpeed;
        if (gamepad2.left_trigger > 0 && clawPosition >= clawMin)
            clawPosition -= clawSpeed;


        clawPosition = Range.clip(clawPosition,clawMin,clawMax);
        claw.setPosition(clawPosition);

        wristPosition = Range.clip(wristPosition,wristMin,wristMax);
        wrist.setPosition(wristPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
        telemetry.addData("Wrist", wristPosition);
        telemetry.addData("Claw", clawPosition);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
