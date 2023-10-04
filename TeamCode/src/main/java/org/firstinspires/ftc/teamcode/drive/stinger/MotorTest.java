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

package org.firstinspires.ftc.teamcode.drive.stinger;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;

/**
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
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Config
@Disabled
@TeleOp(name="MotorTest", group="Test")
public class MotorTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime     runtime     = new ElapsedTime();
    private DcMotor         leftWheel   = null;
    private DcMotor         rightWheel  = null;
    private DcMotor         liftMotor   = null;
    private Servo           claw        = null;
    private Servo           arm         = null;

    static double clawStart              = 0.5;
    public static double clawMinRange          = 0.0;

    public static double clawMaxRange          = 1.0;
    public static double clawSpeed             = 0.001;
    public static double armMinRange          = 0.0;

    public static double armMaxRange          = 0.7;
    public static double armSpeed             = 0.01;
    public static double driveSpeed            = 0.6;
    public static double liftSpeed             = 0.6;
    double clawPosition                       = 0.5;

    double armPosition                         = 0.5;

//    public static double Controls              = 1;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftWheel  = hardwareMap.get(DcMotor.class, "LeftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "RightWheel");
        liftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        claw = hardwareMap.get(Servo.class, "Claw");
        arm = hardwareMap.get(Servo.class, "Arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        claw.setPosition(clawStart);

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
        double LeftPower;
        double RightPower;
        double LiftPower;
        final double Deadzone = 0.1;
        double Drive = 0.0;
        double Turn = 0.0;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

//        if(Controls == 1){
//
//            double DriveInput =  -gamepad1.left_stick_y;
//            if(Math.abs(DriveInput) < Deadzone) {
//                DriveInput = 0.0;
//            }
//            Drive = DriveInput * DriveInput * DriveInput;
//
//            double TurnInput =  -gamepad1.left_stick_x;
//            if(Math.abs(TurnInput) < Deadzone) {
//                TurnInput = 0.0;
//            }
//            Turn = TurnInput * TurnInput * TurnInput;
//        }

//        if(Controls == 2){

            double DriveInput =  -gamepad1.left_stick_y;
            if(Math.abs(DriveInput) < Deadzone) {
                DriveInput = 0.0;
            }
            Drive = DriveInput * DriveInput * DriveInput;

            double TurnInput =  gamepad1.right_stick_x;
            if(Math.abs(TurnInput) < Deadzone) {
                TurnInput = 0.0;
            }
            Turn = TurnInput * TurnInput * TurnInput;
//                   }


        double LiftInput = -gamepad2.left_stick_y;
        if(Math.abs(LiftInput) < Deadzone) {
            LiftInput = 0.0;
        }
        double Lift = LiftInput * LiftInput * LiftInput;

        if (-gamepad2.right_stick_y > 0 && armPosition < armMaxRange)
            armPosition += armSpeed;
        if (-gamepad2.right_stick_y > 0 && armPosition >= armMinRange)
            armPosition -= armSpeed;




        if (gamepad2.right_trigger > 0 && clawPosition < clawMaxRange)
            clawPosition += clawSpeed;
        if (gamepad2.left_trigger > 0 && clawPosition >= clawMinRange)
            clawPosition -= clawSpeed;



        LeftPower    = Range.clip(Drive + Turn, -1.0, 1.0) ;
        RightPower   = Range.clip(Drive - Turn, -1.0, 1.0) ;
        LiftPower    = Range.clip(Lift, -1.0, 1.0);


        // Send calculated power to wheels
        leftWheel.setPower(LeftPower * driveSpeed);
        rightWheel.setPower(RightPower * driveSpeed);
        liftMotor.setPower(LiftPower * liftSpeed);


        // Limit switch code for switches







        // Set servo Position
        clawPosition = Range.clip(clawPosition, clawMinRange, clawMaxRange) ;
        claw.setPosition(clawPosition);
        armPosition = Range.clip(armPosition, armMinRange, armMaxRange) ;
        arm.setPosition(armPosition);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", LeftPower, RightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}