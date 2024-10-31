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

import static org.firstinspires.ftc.teamcode.drive.stinger.StingerConstants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.drive.stinger.StingerConstants.LIFT_COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.drive.stinger.StingerConstants.liftSpeed;
import static org.firstinspires.ftc.teamcode.drive.stinger.StingerConstants.driveSpeed;
import static org.firstinspires.ftc.teamcode.drive.stinger.StingerConstants.turnSpeed;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//code for cam and other related thangs


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, inches, inches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RightAuto", group="Swarm")
//@Disabled
public class RightAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor frontLift = null;

    public static final double  driveSpeed = 0.6;
    public static final double  turnSpeed = 0.5;
    public static final double liftSpeed = 1.0;
    public static final double intakeSpeed = 1;
    public static final double     COUNTS_PER_MOTOR_REV    = 529.2 ;
    public static final double      WHEEL_DIAMETER_INCHES   = 75/25.4 ;     // For figuring circumference
    public static final double      DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.

    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double     STRAFE_INCH_PER_REV     = 0.5;
    public static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            STRAFE_INCH_PER_REV;
    public static final double     DEGREE_PER_REV          = 60;
    public static final double      COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            DEGREE_PER_REV;
    public static final double    LIFT_INCH_PER_REV       = 2.5;
    public static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            LIFT_INCH_PER_REV;


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        frontLift = hardwareMap.get(DcMotor.class, "frontLift");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        frontLift.setDirection(DcMotor.Direction.FORWARD);



        frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /* parameters :
        encoderDrive(DRIVE_SPEED, How many inches you want to move (+ forward - reverse),
        Where you want to set the lift height in inches,
        Whether you want the claw open (1) or closed (0) at the end of the step,
        maximum time allowed for the step before it automatically stops.)

        encoderStrafe(DRIVE_SPEED, How many inches you want to move (+ Right - Left),
        Where you want to set the lift height in inches,
        Whether you want the claw open (1) or closed (0) at the end of the step,
        maximum time allowed for the step before it automatically stops.)

        encoderSpin(DRIVE_SPEED, How many degrees you want to rotate (+ clockwise - counter clockwise),
        Where you want to set the lift height in inches,
        Whether you want the claw open (1) or closed (0) at the end of the step,
        maximum time allowed for the step before it automatically stops.)
         */



          /*
                encoderStrafe(driveSpeed,0,0,0,5.0);
                encoderDrive(driveSpeed,0,0,0,5.0);
                encoderDrive(driveSpeed,0,0,0,5.0);
                encoderDrive(driveSpeed,0,0,0,5.0);
                encoderSpin(turnSpeed,0,0,0,5.0);
                encoderDrive(driveSpeed, 0,0,0,5.0);
                encoderStrafe(driveSpeed,0,0,0,5.0);
                encoderDrive(driveSpeed, 0,0,0,5.0);
                encoderStrafe(driveSpeed,0,0,0,5.0);
                encoderDrive(driveSpeed, 0,0,0, 5.0);
                sleep(26000);
          */
        //Skibidi Left Auto
       // encoderDrive(driveSpeed, 24, 5.0);
       // encoderSpin(turnSpeed,-90,5.0);
        //Place purple pixel on spike mark
        //encoderDrive(driveSpeed, -36, 5.0);
        //encoderStrafe(driveSpeed,6,5.0);
        //Place yellow pixel on back drop
        //encoderStrafe(driveSpeed, 18, 5.0);
        //encoderDrive(driveSpeed, -12, 5.0);


//        encoderDrive(driveSpeed, 24,16, 5.0);
//        encoderLift(liftSpeed,0,5.0);
//        //Place specimen on bar
//        encoderStrafe(driveSpeed,32,0,5.0);
//        encoderSpin(turnSpeed,180,0,5.0);
//        encoderDrive(driveSpeed, 22,0,5.0);
//        encoderLift(liftSpeed,14,5.0);
//        //Take the specimen
//        encoderDrive(driveSpeed,-12,16,5.0);
//        encoderSpin(turnSpeed, 180,16, 5.0);
//        encoderStrafe(driveSpeed, -41,16, 5.0);
//        encoderDrive(driveSpeed,11,16,5.0);
//        encoderLift(liftSpeed,0,5.0);
//        //Place specimen on bar
//        encoderDrive(driveSpeed, -8, 0,5.0);
//        encoderSpin(turnSpeed, 180,0, 5.0);
//        encoderStrafe(driveSpeed, -43, 0,5.0);
//        encoderDrive(driveSpeed,10,0,5.0);
//        encoderLift(liftSpeed,16,5.0);
//        //Take the specimen
//        encoderDrive(driveSpeed,-10,16,5.0);
//        encoderSpin(turnSpeed, 180, 16,5.0);
//        encoderStrafe(driveSpeed, -45, 16,5.0);
//        encoderDrive(driveSpeed,15,16,5.0);
//        encoderLift(liftSpeed,0,5.0);
//        //Place the specimen on bar
//        encoderStrafe(driveSpeed, 57, 0,5.0);
//        encoderDrive(driveSpeed, -30, 0,5.0);
        //Park

        encoderSpin(turnSpeed, 360, 0,5.0);

    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    public void encoderDrive(double speed,
                             double inches,
                             double frontLiftInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newFrontLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLiftTarget = (int)(frontLiftInches * LIFT_COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            frontLift.setTargetPosition(newFrontLiftTarget);
            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            frontLift.setPower(liftSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()
                    )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            frontLift.setPower(0);


            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }


    public void encoderStrafe(double speed,
                              double inches,
                              double frontLiftInches,
                              double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newFrontLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newFrontLiftTarget = (int)(frontLiftInches * LIFT_COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            frontLift.setTargetPosition(newFrontLiftTarget);
            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            frontLift.setPower(liftSpeed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()
                    )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            frontLift.setPower(0);


            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderSpin(double speed,
                            double degress,
                            double frontLiftInches,
                            double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newFrontLiftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(degress* COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() - (int)(degress * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(degress * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(degress * COUNTS_PER_INCH);
            newFrontLiftTarget = (int)(frontLiftInches * LIFT_COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            frontLift.setTargetPosition(newFrontLiftTarget);
            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            frontLift.setPower(liftSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()
                    )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            frontLift.setPower(0);


            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderLift(double liftSpeed,
                            int frontLiftInches,
                            double timeoutS) {

        int newFrontLiftTarget;

        if (opModeIsActive()) {

            newFrontLiftTarget = (int)(frontLiftInches * LIFT_COUNTS_PER_INCH);

            frontLift.setTargetPosition(newFrontLiftTarget);

            frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            frontLift.setPower(liftSpeed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLift.isBusy())) {


            }
            frontLift.setPower(0);



            sleep(250);

        }
    }


}
