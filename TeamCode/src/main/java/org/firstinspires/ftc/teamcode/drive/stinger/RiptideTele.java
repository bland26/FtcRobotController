package org.firstinspires.ftc.teamcode.drive.stinger;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//you can has cheezburger


@TeleOp(name="RiptideTele", group="Swarm")
@Config

//@Disabled
public class RiptideTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor lift = null;
    private DcMotor arm = null;
    private DcMotor climbRight = null;
    private DcMotor climbLeft = null;
    private CRServo intake = null;
    private TouchSensor liftLimit = null;
    private TouchSensor armLimit = null;


    //TODO Decide names for and declare extra motors. (Top intake, bottom intake, lift)
    //TODO Decide names for and declare servos.


    public static double driveSpeed = 1.0;

    public static double liftSpeed = 1.0;
    public static double climbSpeed = 1.0;

    public double encoderOverride = 0;

    public static final double     COUNTS_PER_MOTOR_REV    = 529.2 ;
    public static final double      WHEEL_DIAMETER_INCHES   = 75/25.4 ;     // For figuring circumference
    public static final double      DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.

    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double     STRAFE_INCH_PER_REV     = 0.5;
    public static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            STRAFE_INCH_PER_REV;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(DcMotor.class, "arm");
        climbRight = hardwareMap.get(DcMotor.class, "climbRight");
        climbLeft = hardwareMap.get(DcMotor.class, "climbLeft");
        intake = hardwareMap.get(CRServo.class, "intake");
        liftLimit = hardwareMap.get(TouchSensor.class, "backLiftLimit");
        armLimit = hardwareMap.get(TouchSensor.class, "armLimit");


        //TODO initilize new motors that were added

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE); // slide
        arm.setDirection(DcMotor.Direction.REVERSE);
        climbRight.setDirection(DcMotor.Direction.FORWARD);
        climbLeft.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(CRServo.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);






        //TODO set new motor directions

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
        double leftRearPower;
        double rightRearPower;
        double leftFrontPower;
        double rightFrontPower;
        double liftPower;
        double armPower;
        double climbPower;
        double intakePower;
        //TODO initilize New Motor power variables


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        final double DEADZONE = 0.1;

        double drive = -gamepad1.left_stick_y;
        if (Math.abs(drive) < DEADZONE) {
            drive = 0;
        }
        double driveCubed = drive * drive * drive;

        double strafe = gamepad1.left_stick_x;
        if (Math.abs(strafe) < DEADZONE) {
            strafe = 0;
        }
        double strafeCubed = strafe * strafe * strafe;

        double spin = gamepad1.right_stick_x;
        if (Math.abs(spin) < DEADZONE) {
            spin = 0;
        }
        double spinCubed = spin * spin * spin;


        double backLiftInput = -gamepad2.left_stick_y;
        if (Math.abs(backLiftInput) < DEADZONE) {
            backLiftInput = 0;
        }
        double backLiftCubed = backLiftInput * backLiftInput * backLiftInput;

        double armInput = -gamepad2.right_stick_y;
        if (Math.abs(armInput) < DEADZONE) {
            armInput = 0;
        }
        double armCubed = armInput * armInput * armInput;


        float intakeInput = gamepad2.right_trigger;
        if (intakeInput > 0){
            intakePower=1;
        }else {
            intakePower=0;
        }

        float intakeInputR = gamepad2.left_trigger;
        if (intakeInputR > 0){
            intakePower = -1;
        }
/*
        if ( gamepad2.right_bumper  ) {
            climbPower = -climbSpeed;
        } else if ((gamepad2.left_bumper & climbLeft.getCurrentPosition() < 0) || (gamepad2.left_bumper & gamepad2.y)) {
            climbPower =  climbSpeed;

        } else {
            climbPower = 0;
        }
*/
        if ( gamepad2.right_bumper  ) {
            encoderStrafe(1,2,1);
        }
        if( gamepad2.left_bumper){
            encoderStrafe(1,-2,1);
        }





        //TODO create methods for new motors

       /* if (!scoreCon) {
            leftRearPower = Range.clip(driveCubed + spinCubed - strafeCubed, -1.0, 1.0);
            rightRearPower = Range.clip(driveCubed - spinCubed + strafeCubed, -1.0, 1.0);
            leftFrontPower = Range.clip(driveCubed + spinCubed + strafeCubed, -1.0, 1.0);
            rightFrontPower = Range.clip(driveCubed - spinCubed - strafeCubed, -1.0, 1.0);
            frontLiftPower = Range.clip(frontLiftCubed, -1.0, 1.0);

        }else {

            leftRearPower = Range.clip((driveCubed) + spinCubed - (strafeCubed), -0.5, 0.5);
            rightRearPower = Range.clip((driveCubed)- spinCubed + (strafeCubed), -0.5, 0.5);
            leftFrontPower = Range.clip((driveCubed) + spinCubed + (strafeCubed), -0.5, 0.5);
            rightFrontPower = Range.clip((driveCubed) - spinCubed - (strafeCubed), -0.5, 0.5);
            frontLiftPower = Range.clip(frontLiftCubed, -1.0, 1.0);

        }*/

        leftRearPower = Range.clip(driveCubed + spinCubed - strafeCubed, -1.0, 1.0);
        rightRearPower = Range.clip(driveCubed - spinCubed + strafeCubed, -1.0, 1.0);
        leftFrontPower = Range.clip(driveCubed + spinCubed + strafeCubed, -1.0, 1.0);
        rightFrontPower = Range.clip(driveCubed - spinCubed - strafeCubed, -1.0, 1.0);
        liftPower = Range.clip(backLiftCubed, -1.0, 1.0);
        armPower = Range.clip(armCubed, -1.0, 1.0);

        if (gamepad2.x){
            encoderOverride =  -arm.getCurrentPosition();
        }


        if ( gamepad2.dpad_up) {
            if (arm.getCurrentPosition() + encoderOverride < 3100) {
                arm.setPower(liftSpeed);
            } else if (arm.getCurrentPosition()  + encoderOverride < 3200) {
                arm.setPower(liftSpeed * 0.5);
            } else {
                arm.setPower(0);
            }
            if (lift.getCurrentPosition() < 850 && arm.getCurrentPosition()  + encoderOverride > 2200) {
                lift.setPower(liftSpeed);
            } else {
                lift.setPower(0);
            }
        } else if ( gamepad2.dpad_right) {
            if (arm.getCurrentPosition()  + encoderOverride < 1900) {
                arm.setPower(liftSpeed);
            } else if (arm.getCurrentPosition()  + encoderOverride < 2000) {
                arm.setPower(liftSpeed*0.5);
            } else {
                arm.setPower(0);
            }
//            if (lift.getCurrentPosition() < 1662 && arm.getCurrentPosition()  + encoderOverride > 1300) {
//                lift.setPower(liftSpeed);
//            } else {
//                lift.setPower(0);
//            }
        } else if(gamepad2.dpad_down){
            if (arm.getCurrentPosition()  + encoderOverride > 0 && lift.getCurrentPosition() < 500) {
                arm.setPower(-liftSpeed);
            } else {
                arm.setPower(0);
            }
            if (lift.getCurrentPosition() > 0) {
                lift.setPower(-liftSpeed);
            } else {
                lift.setPower(0);
            }
        } else {

            if(arm.getCurrentPosition()  + encoderOverride > 3240 && armPower > 0){
                arm.setPower(0);
            } else{
                arm.setPower(armPower* liftSpeed);
            }
            if ((liftPower < 0 && liftLimit.isPressed()) || (liftPower > 0 && lift.getCurrentPosition() > 2700)) {
                lift.setPower(0);
            } else {
                lift.setPower(liftPower * liftSpeed);
            }
        }


//        boolean climb = gamepad2.right_bumper;
//        if (climb){
//            climbLeft.setTargetPosition(-5000);
//            climbRight.setTargetPosition(-5000);
//            //arm.setTargetPosition(2000);
//            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            climbLeft.setPower(1);
//            climbRight.setPower(1);
//            //arm.setPower(1);
//        }else {
//            climbPower=0;
//        }
//
//        boolean climbReverse = gamepad2.left_bumper;
//        if (climbReverse){
//            climbLeft.setTargetPosition(0);
//            climbRight.setTargetPosition(0);
//           // arm.setTargetPosition(1000);
//            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           // arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            climbLeft.setPower(1);
//            climbRight.setPower(1);
//           // arm.setPower(1);
//        }
/*
        if ( gamepad2.right_bumper & climbLeft.getCurrentPosition() < -5000) {
            climbLeft.setPower(climbSpeed);
            climbRight.setPower(climbSpeed);
            } else {
                climbLeft.setPower(0);
                climbRight.setPower(0);
            }
        } else if (gamepad2.left_bumper) {
            if (climbLeft.getCurrentPosition() > 0) {
                climbLeft.setPower(-climbSpeed);
                climbRight.setPower(-climbSpeed);
            } else {
                climbLeft.setPower(0);
                climbRight.setPower(0);
            }
        }
*/



        // Send calculated power to wheels

        leftRear.setPower(leftRearPower * driveSpeed);
        rightRear.setPower(rightRearPower * driveSpeed);
        leftFront.setPower(leftFrontPower * driveSpeed);
        rightFront.setPower(rightFrontPower * driveSpeed);
        //climbRight.setPower(climbPower);
        //climbLeft.setPower(climbPower);
        intake.setPower(intakePower);



//        if (sensorDistance.getDistance(DistanceUnit.MM) < 5 && scoreCon && leftFrontPower > 0){
//            leftRear.setPower(0);
//            rightRear.setPower(0);
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//        }



//        if (liftPower < 0 && backLiftLimit.isPressed()) {
//            backLift.setPower(0);
//        } else {
//            backLift.setPower(liftPower * liftSpeed);
//        }
//        if (armPower < 0 && armLimit.isPressed()) {
//            arm.setPower(0);
//        } else {
//            arm.setPower(armPower * liftSpeed);
//        }

        //TODO set new motor power


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftRearPower, rightRearPower);
        telemetry.addData("right", climbRight.getCurrentPosition());
        telemetry.addData("left", climbLeft.getCurrentPosition());
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("slide", lift.getCurrentPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public void encoderStrafe(double speed,
                              double inches,

                              double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;




            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftRear.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRightBackTarget = rightRear.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

            leftRear.setTargetPosition(newLeftBackTarget);
            rightRear.setTargetPosition(newRightBackTarget);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (
                    (runtime.seconds() < timeoutS) &&
                    (leftRear.isBusy() && rightRear.isBusy() && leftFront.isBusy() && rightFront.isBusy()
                    )) {



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

        }

}

