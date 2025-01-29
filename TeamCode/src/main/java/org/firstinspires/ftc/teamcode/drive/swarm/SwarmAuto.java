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

package org.firstinspires.ftc.teamcode.drive.swarm;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

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

@Autonomous(name="SwarmAuto", group="Swarm")
//@Disabled

public class SwarmAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor extension = null;
    private DcMotor pivot = null;
    private CRServo intake = null;

    public static final double  driveSpeed = 0.8;
    public static final double  turnSpeed = 0.5;
    public static final double extensionSpeed = 1.0;
    public static final double intakeSpeed = 1;
    public static final double     COUNTS_PER_MOTOR_REV    = 529.2 ;
    public static final double      WHEEL_DIAMETER_INCHES   = 75/25.4 ;     // For figuring circumference
    public static final double      DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.

    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double     STRAFE_INCH_PER_REV     = 0.5;
    public static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            STRAFE_INCH_PER_REV;
    public static final double     DEGREE_PER_REV          = 37.75;
    public static final double      COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            DEGREE_PER_REV;
    public static final double EXTENSION_INCH_PER_REV = 2.1;
    public static final double EXTENSION_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            EXTENSION_INCH_PER_REV;
    public static final double PIVOT_INCH_PER_REV = 3;
    public static final double PIVOT_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            PIVOT_INCH_PER_REV;



    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        intake = hardwareMap.get(CRServo.class, "intake");




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        extension.setDirection(DcMotor.Direction.FORWARD);
        pivot.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(CRServo.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
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



//        encoderDrive(driveSpeed,24,0,0,0,5.0);
//        encoderLift(liftSpeed,0,0,0,5.0);
//        encoderStrafe(driveSpeed,-38.5,0,0,0,5.0);
//        encoderIntake(intakeSpeed,1,5.0);
//        encoderSpin(turnSpeed,-135,0,0,0,5.0);
//        encoderLift(0,0,0,0,5.0);
//        encoderIntake(intakeSpeed,-1,5.0);
//        encoderSpin(turnSpeed,135,0,0,0,5.0);
//        encoderStrafe(driveSpeed,-12,0,0,0,5.0);
//        encoderIntake(intakeSpeed,1,5.0);
//        encoderStrafe(driveSpeed,12,0,0,0,5.0);
//        encoderSpin(turnSpeed,-135,0,0,0,5.0);
//        encoderLift(0,0,0,0,5.0);
//        encoderIntake(intakeSpeed,-1,5.0);
//        encoderLift(0,0,0,0,5.0);
//        encoderSpin(turnSpeed,-45,0,0,0,5.0);
//        encoderDrive(driveSpeed,20,0,0,0,5.0);
//        encoderStrafe(driveSpeed,-110,0,0,0,5.0);

        encoderlift(extensionSpeed, 0, 1000, 0, 2.0);
        encoderDrive(driveSpeed,27,0,1500,0,5.0);
        encoderlift(extensionSpeed, 0, 1000, 0, 5.0);
        encoderDrive(driveSpeed,-10,0,500,0,5.0);
        encoderStrafe(driveSpeed, 31, 0, 500, 0, 5.0);
        encoderDrive(driveSpeed, 32, 0, 500, 0, 5.0);
        encoderStrafe(driveSpeed, 10,0,500,0,5.0);
        encoderDrive(driveSpeed, -40,0,500,0,5.0);
        encoderDrive(driveSpeed,12,0,500,0,5.0);
        encoderSpin(turnSpeed, 175, 0, 0, 0, 5.0);
        sleep(1000);
        encoderDrive(driveSpeed/2,12,400,0,1,5.0);
        encoderDrive(driveSpeed, -8, 0, 500, 1, 5.0);
        encoderSpin(turnSpeed, 170, 0, 500, 1, 5.0);
        encoderStrafe(driveSpeed, -48, 0, 1000, 1, 5.0);
        encoderDrive(driveSpeed, 14, 0, 1417, 1, 5.0);
        encoderlift(extensionSpeed, 0, 1000, 0, 5.0);
        encoderDrive(driveSpeed, -24, 0, 0, 0, 5.0);
        encoderStrafe(driveSpeed, 48, 0, 0, 0, 5.0);

        /*
        encoderStrafe(driveSpeed, 48, 0, 0, 0, 5.0);
        encoderSpin(turnSpeed, 180, 0, 0, 0, 5.0);
        encoderDrive(driveSpeed/2, 24, 0, 0, 1, 5.0);
        encoderDrive(driveSpeed, -24, 0, 0, 0, 5.0);
        encoderSpin(turnSpeed, 180, 0, 0, 0, 5.0);
        encoderStrafe(driveSpeed, -52, 0, 0, 0, 5.0);
        encoderlift(extensionSpeed, 0, 1000, 0, 2.0);
        encoderDrive(driveSpeed, 4, 0, 1417, 0, 5.0);
        encoderlift(extensionSpeed, 0, 1000, 0, 5.0);
        encoderDrive(driveSpeed, -4, 0, 0, 0, 5.0);
        encoderStrafe(driveSpeed, 52, 0, 0, 0, 5.0);
        encoderSpin(turnSpeed, 180, 0, 0, 0, 5.0);
        encoderDrive(driveSpeed/2, 24, 0, 0, 1, 5.0);
        encoderDrive(driveSpeed, -24, 0, 0, 0, 5.0);
        encoderSpin(turnSpeed, 180, 0, 0, 0, 5.0);
        encoderStrafe(driveSpeed, -56, 0, 0, 0, 5.0);
        encoderlift(extensionSpeed, 0, 1000, 0, 2.0);
        encoderDrive(driveSpeed, 4, 0, 1417, 0, 5.0);
        encoderlift(extensionSpeed, 0, 1000, 0, 5.0);
        encoderDrive(driveSpeed, -24, 0, 0, 0, 5.0);
        encoderStrafe(driveSpeed, -60, 0, 0, 0, 5.0);
         */



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
                             double extensionInches,
                             double pivotInches,
                             int intakeValue,
                             double timeoutS) {
        int newLeftBackDriveTarget;
        int newRightBackDriveTarget;
        int newLeftFrontDriveTarget;
        int newRightFrontDriveTarget;
        int newExtensionTarget;
        int newPivotTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackDriveTarget = leftBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackDriveTarget = rightBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontDriveTarget = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontDriveTarget = rightFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newExtensionTarget = (int)(extensionInches);
            newPivotTarget = (int)(pivotInches);
            leftBackDrive.setTargetPosition(newLeftBackDriveTarget);
            rightBackDrive.setTargetPosition(newRightBackDriveTarget);
            leftFrontDrive.setTargetPosition(newLeftFrontDriveTarget);
            rightFrontDrive.setTargetPosition(newRightFrontDriveTarget);
            extension.setTargetPosition(newExtensionTarget);
            pivot.setTargetPosition(newPivotTarget);
            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            extension.setPower(extensionSpeed);
            pivot.setPower(extensionSpeed);
            intake.setPower(intakeValue);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                    )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackDriveTarget,  newRightBackDriveTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontDriveTarget,  newRightFrontDriveTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            extension.setPower(0);
            pivot.setPower(0);
            intake.setPower(0);


            // Turn off RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }


    public void encoderStrafe(double speed,
                              double inches,
                              double extensionInches,
                              double pivotInches,
                              int intakeValue,
                              double timeoutS) {
        int newLeftBackDriveTarget;
        int newRightBackDriveTarget;
        int newLeftFrontDriveTarget;
        int newRightFrontDriveTarget;
        int newExtensionTarget;
        int newPivotTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackDriveTarget = leftBackDrive.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackDriveTarget = rightBackDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontDriveTarget = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontDriveTarget = rightFrontDrive.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newExtensionTarget = (int)(extensionInches);
            newPivotTarget = (int)(pivotInches);
            leftBackDrive.setTargetPosition(newLeftBackDriveTarget);
            rightBackDrive.setTargetPosition(newRightBackDriveTarget);
            leftFrontDrive.setTargetPosition(newLeftFrontDriveTarget);
            rightFrontDrive.setTargetPosition(newRightFrontDriveTarget);
            extension.setTargetPosition(newExtensionTarget);
            pivot.setTargetPosition(newPivotTarget);
            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            extension.setPower(extensionSpeed);
            pivot.setPower(extensionSpeed);
            intake.setPower(intakeValue);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                    )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackDriveTarget,  newRightBackDriveTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontDriveTarget,  newRightFrontDriveTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            extension.setPower(0);
            pivot.setPower(0);
            intake.setPower(0);


            // Turn off RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderSpin(double speed,
                            double degrees,
                            double extensionInches,
                            double pivotInches,
                            int intakeValue,
                            double timeoutS) {
        int newLeftBackDriveTarget;
        int newRightBackDriveTarget;
        int newLeftFrontDriveTarget;
        int newRightFrontDriveTarget;
        int newExtensionTarget;
        int newPivotTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackDriveTarget = leftBackDrive.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newRightBackDriveTarget = rightBackDrive.getCurrentPosition() - (int)(degrees * COUNTS_PER_DEGREE);
            newLeftFrontDriveTarget = leftFrontDrive.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newRightFrontDriveTarget = rightFrontDrive.getCurrentPosition() - (int)(degrees * COUNTS_PER_DEGREE);
            newExtensionTarget = (int)(extensionInches);
            newPivotTarget = (int)(pivotInches);
            leftBackDrive.setTargetPosition(newLeftBackDriveTarget);
            rightBackDrive.setTargetPosition(newRightBackDriveTarget);
            leftFrontDrive.setTargetPosition(newLeftFrontDriveTarget);
            rightFrontDrive.setTargetPosition(newRightFrontDriveTarget);
            extension.setTargetPosition(newExtensionTarget);
            pivot.setTargetPosition(newPivotTarget);
            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            extension.setPower(extensionSpeed);
            pivot.setPower(extensionSpeed);
            intake.setPower(intakeValue);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                    )) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackDriveTarget,  newRightBackDriveTarget);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontDriveTarget,  newRightFrontDriveTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            extension.setPower(0);
            pivot.setPower(0);
            intake.setPower(0);


            // Turn off RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderlift(double extensionSpeed,
                                 double extensionInches,
                                 double pivotInches,
                                 int intakeValue,
                                 double timeoutS) {

        int newExtensionTarget;
        int newPivotTarget;

        if (opModeIsActive()) {

            newExtensionTarget = (int)(extensionInches);
            newPivotTarget = (int)(pivotInches);

            extension.setTargetPosition(newExtensionTarget);
            pivot.setTargetPosition(newPivotTarget);

            extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            extension.setPower(extensionSpeed);
            pivot.setPower(extensionSpeed);
            intake.setPower(intakeValue);


            while (opModeIsActive() &&
                    ((runtime.seconds() < timeoutS) && (pivot.isBusy() || extension.isBusy()))){

                telemetry.addData("Status", "Run Time: " + runtime.toString());

                telemetry.addData("Pivot", pivot.getCurrentPosition());
                telemetry.addData("Extension", extension.getCurrentPosition());


            }
            extension.setPower(0);
            pivot.setPower(0);
            intake.setPower(0);



            sleep(250);

        }
    }
    private void encoderIntake(int intakeValue,
                               double timeoutS) {


        if (opModeIsActive()) {


            runtime.reset();
            intake.setPower(intakeValue);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {


            }
            intake.setPower(0);


            sleep(250);
        }
    }


}
