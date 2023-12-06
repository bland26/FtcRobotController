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

package org.firstinspires.ftc.teamcode.drive.hive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import static org.firstinspires.ftc.teamcode.drive.hive.HiveConstants.*;

import java.util.List;

//code for cam and other related thangs
//hola guys
//Abi has a secret

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

@Autonomous(name="HiveAutoFrontRed", group="Hive")
@Config
@Disabled
public class HiveAutoFrontRed extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor lift = null;
    private DcMotor intake = null;
    private CRServo outtake = null;

    private  TouchSensor limitDown;

    private Servo claw = null;


    final static double clawStart = 0.1;
    public static double clawMin = 0.0;
    public static double clawMax = 0.25;
    public static double clawSpeed = 0.01;
    public double clawPosition = 0.0;



    public static double driveSpeed = 0.8;

    public static double liftSpeed = 1.0;

    public static double intakeSpeed = 0.5;


    public double x = 500;
    private double y = 0;

    private String path = null;



    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        limitDown = hardwareMap.get(TouchSensor.class, "limitDown");
        claw = hardwareMap.get(Servo.class, "claw");





        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        claw.setPosition(clawPosition);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        initTfod();
        targetTfod();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
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

        if (x >= 200 && x < 720) { // Middle Path
            path = "1";
            encoderStrafe(driveSpeed,2,0,0,0,0,5.0);
            encoderDrive(driveSpeed,20,0,-1,0,0,5.0);
            encoderDrive(driveSpeed,4,0,1,0,1,5.0);
            encoderSpin(turnSpeed, -90,0,1,0,1,5.0);
            encoderDrive(driveSpeed,-40,3,0,0, 1,5.0);
            encoderDrive(driveSpeed,-43,3,0,0, 1,5.0);
            encoderStrafe(driveSpeed,2,3,0,0,0.5,5.0);
            score(-1,2.0);
            encoderStrafe(driveSpeed, 26,0,0,0,0.25,5.0);
            encoderDrive(driveSpeed, -8,0,0,0,0.25,5.0);
            sleep(20000);
        } else if (x > 200) { // Right Path
            path = "2";
            encoderStrafe(driveSpeed,2,0,0,0,0,5.0);
            encoderDrive(driveSpeed,24,0,-1,0,0,5.0);
            encoderSpin(turnSpeed, -90,0,0,0,0,5.0);
            encoderDrive(driveSpeed,-20,0,0,0,0,5.0);
            encoderDrive(driveSpeed,-20,0,1,0, 1,5.0);
            encoderDrive(driveSpeed,-43,3,0,0, 1,5.0);
            encoderStrafe(driveSpeed,6,3,0,0,0.5,5.0);
            score(-1,2.0);
            encoderStrafe(driveSpeed, 22,0,0,0,0.25,5.0);
            encoderDrive(driveSpeed, -8,0,0,0,0.25,5.0);
            sleep(20000);
        } else { // Left Path
            path = "3";
            encoderStrafe(driveSpeed,2,0,0,0,0,5.0);
            encoderDrive(driveSpeed,24,3,-1,0,0,5.0);
            encoderSpin(turnSpeed, -90,3,0,0,0,5.0);
            encoderDrive(driveSpeed,4,3,0,0,1,5.0);
            encoderDrive(driveSpeed,-44,3,1,0, 1,5.0);
            encoderDrive(driveSpeed,-43,3,0,0, 1,5.0);
            encoderStrafe(driveSpeed,-2,3,0,0,0.5,5.0);
            score(-1,2.0);
            encoderStrafe(driveSpeed, 30,0,0,0,0.25,5.0);
            encoderDrive(driveSpeed, -8,0,0,0,0.25,5.0);
            sleep(20000);
        }




        telemetry.addData("Path", path);
        telemetry.addData("position", "%.0f", x);
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
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
                             double liftInches,
                             double intakeValue,
                             double outtakeValue,
                             double clawValue,
                             double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLiftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLiftTarget = (int)(liftInches * LIFT_COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            lift.setTargetPosition(newLiftTarget);
            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            lift.setPower(liftSpeed);
            intake.setPower(intakeValue);
            outtake.setPower(outtakeValue);
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
            lift.setPower(0);


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
                              double liftInches,
                              double intakeValue,
                              double outtakeValue,
                              double clawValue,
                              double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLiftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() - (int)(inches * STRAFE_COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * STRAFE_COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * STRAFE_COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * STRAFE_COUNTS_PER_INCH);
            newLiftTarget = (int)(liftInches * LIFT_COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            lift.setTargetPosition(newLiftTarget);
            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            lift.setPower(liftSpeed);
            intake.setPower(intakeValue);
            outtake.setPower(outtakeValue);
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
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            lift.setPower(0);



            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderSpin(double speed,
                            double degrees,
                            double liftInches,
                            double intakeValue,
                            double outtakeValue,
                            double clawValue,
                            double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLiftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newRightBackTarget = rightRear.getCurrentPosition() - (int)(degrees * COUNTS_PER_DEGREE);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(degrees * COUNTS_PER_DEGREE);
            newLiftTarget = (int)(liftInches * LIFT_COUNTS_PER_INCH);
            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            lift.setTargetPosition(newLiftTarget);
            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            lift.setPower(liftSpeed);
            intake.setPower(intakeValue);
            outtake.setPower(outtakeValue);
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
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftRear.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            lift.setPower(0);



            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderLift(double speed,
                            int liftInches,
                            double clawValue,
                            double timeoutS) {

        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = (int)(liftInches * LIFT_COUNTS_PER_INCH);

            lift.setTargetPosition(newLiftTarget);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            lift.setPower(liftSpeed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lift.isBusy())) {


            }
            lift.setPower(0);



            sleep(250);

        }
    }
    private void score(int outtakeValue, double timeoutS) {

        outtake.setPower(outtakeValue);

        if (opModeIsActive()) {


            runtime.reset();
            lift.setPower(liftSpeed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {


            }
            outtake.setPower(0);


            sleep(250);
        }
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private double targetTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            return(y);



        }   // end for() loop
        return(y);
    }   // end method telemetryTfod()

}

