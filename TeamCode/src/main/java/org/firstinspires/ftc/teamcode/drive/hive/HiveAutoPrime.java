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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.teamcode.drive.hive.HiveConstants.*;
import static org.firstinspires.ftc.teamcode.drive.swarm.SwarmConstants.COUNTS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.drive.swarm.SwarmConstants.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.drive.swarm.SwarmConstants.LIFT_COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.drive.swarm.SwarmConstants.STRAFE_COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.drive.swarm.SwarmConstants.liftSpeed;

import java.util.List;

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

@Autonomous(name="HiveAutoPrime", group="Hive")

public class HiveAutoPrime extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor lift = null;
    private Servo claw = null;
    private DigitalChannel topLimit = null;
    private TouchSensor botLimit = null;

    private ElapsedTime     runtime = new ElapsedTime();


    // Object detection variables
    //private static final String TFOD_MODEL_ASSET = "model_20221206_120301.tflite";
    private static final String TFOD_MODEL_FILE  = "model_20221206_120301.tflite";
    private static final String[] LABELS = {
            "Circle",
            "Square",
            "Triangle"
    };

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "Lift");
        claw = hardwareMap.get(Servo.class, "Claw");
        topLimit = hardwareMap.get(DigitalChannel.class, "TopLimit");
        botLimit = hardwareMap.get(TouchSensor.class, "BotLimit");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setPosition(clawMax);
        topLimit.setMode(DigitalChannel.Mode.INPUT);


        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0);
        }
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
        List<Recognition> objectDetection;
        objectDetection = tfod.getRecognitions();


        for (Recognition object : objectDetection) {
            if (object.getLabel().equals("Triangle")) { // Object 1 path
                encoderDrive(driveSpeed, 0, 0, 0, 5.0);
                encoderDrive(driveSpeed, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(driveSpeed, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(turnSpeed, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(driveSpeed, -42, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(driveSpeed, 3, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                encoderLift(liftSpeed,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(driveSpeed, -3, 36, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(driveSpeed, -12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                encoderDrive(driveSpeed, -43, 5, 1, 5.0); // move backwards 48 inches, set lift to 5 inches, keep claw open.
                sleep(20000);
            } else if (object.getLabel().equals("Circle")) { // Object 2 path
                encoderDrive(driveSpeed, 0, 0, 0, 5.0);
                encoderDrive(driveSpeed, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(driveSpeed, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(turnSpeed, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(driveSpeed, -42, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(driveSpeed, 3, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                encoderLift(liftSpeed,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(driveSpeed, -3, 36, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(driveSpeed, -12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                encoderDrive(driveSpeed, -22, 5, 1, 5.0); // move backwards 24 inches, set lift to 5 inches, keep claw open.
                sleep(20000);
            } else if (object.getLabel().equals("Square")) { // Object 3 path
                encoderDrive(driveSpeed, 0, 0, 0, 5.0);
                encoderDrive(driveSpeed, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(driveSpeed, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(turnSpeed, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(driveSpeed, -42, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(driveSpeed, 3, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                encoderLift(liftSpeed,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(driveSpeed, -3, 36, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(driveSpeed, -12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                sleep(20000);
            } else {
                encoderDrive(driveSpeed, 0, 0, 0, 5.0);
                encoderDrive(driveSpeed, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(driveSpeed, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(turnSpeed, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(driveSpeed, -42, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(driveSpeed, 3, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                encoderLift(liftSpeed,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(driveSpeed, -3, 36, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(driveSpeed, -12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                sleep(20000);
            }
        }

        encoderDrive(driveSpeed, 0, 0, 0, 5.0);
        encoderDrive(driveSpeed, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
        encoderStrafe(driveSpeed, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
        encoderSpin(turnSpeed, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
        encoderStrafe(driveSpeed, -42, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
        encoderDrive(driveSpeed, 3, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
        encoderLift(liftSpeed,30,1, 5.0); // set lift to 30 inches, open claw.
        encoderDrive(driveSpeed, -3, 36, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
        encoderStrafe(driveSpeed, -12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
        sleep(20000);

        telemetry.addData("Path", "Complete");
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
    private static final String VUFORIA_KEY =
            "AZJC/3T/////AAABmRPn6kJC6k6BmqQQ09BqPMdxzm82RZmhCzQAUffgUDxWqKsnQDlYnQFZtG4Flyw/K/G5bXw" +
                    "Wa8z4LXQTxjqjga40ZY3pp73399DYqjOK6jl2BJl5uBss7OHkvUEDlw5kyWoU6xoSfPfNkMJt3Vg2JBl" +
                    "8CDGzXJkuzlGdqo5Hzb48A+8tf3kQv5xvCm90OMSxy48dRFRSfTX+o0yh0NT8l2ihKJ52TJlwYBXnxUD" +
                    "Q7jGInINMZ+SelJe/RCssYGf/YhEKDeqhCTGiBG9Lv9p1K/GFqdnVcRUtkkEoISSO8NnICbVzTTvG+jn" +
                    "2mUUCACNTKwiN6l4lyduZaj9Y3IQ5ioExnW/rdUeat3FEaAVx8vKc";

    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */


    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /*
     * Initialize the Vuforia localization engine.
     */


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.8f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 320;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void clawState(int state){
        if (state == 1){
            claw.setPosition(clawMax);

        }
        else{
            claw.setPosition(clawMin);
        }
    }
    public void encoderDrive(double speed,
                             double inches,
                             int liftInches, int claw,
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

                if (topLimit.getState() == false){
                    lift.setPower(0);
                }if (botLimit.isPressed() == true){
                    lift.setPower(0);
                }
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

            clawState(claw);

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void testDrive(double speed,
                          double inches,

                          double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);

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
                              int liftInches, int claw,
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

                if (topLimit.getState() == false){
                    lift.setPower(0);
                }if (botLimit.isPressed() == true){
                    lift.setPower(0);
                }
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

            clawState(claw);

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void testStrafe(double speed,
                           double inches,
                           double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() - (int)(inches * STRAFE_COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * STRAFE_COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * STRAFE_COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * STRAFE_COUNTS_PER_INCH);

            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);

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




            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderSpin(double speed,
                            double inches,
                            double liftInches,
                            int claw,
                            double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLiftTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_DEGREE);
            newRightBackTarget = rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_DEGREE);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_DEGREE);
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
                if (topLimit.getState() == false){
                    lift.setPower(0);
                }if (botLimit.isPressed() == true){
                    lift.setPower(0);
                }


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

            clawState(claw);

            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void testSpin(double speed,
                         double inches,
                         double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() + (int)(inches * COUNTS_PER_DEGREE);
            newRightBackTarget = rightRear.getCurrentPosition() - (int)(inches * COUNTS_PER_DEGREE);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_DEGREE);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_DEGREE);

            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);

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


            // Turn off RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderLift(double speed,
                            int liftInches, int claw,
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

                if (topLimit.getState() == false){
                    lift.setPower(0);
                }if (botLimit.isPressed() == true){
                    lift.setPower(0);
                }
            }
            lift.setPower(0);

            clawState(claw);

            sleep(250);

        }
    }
}

