package org.firstinspires.ftc.teamcode.drive.stinger;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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



@TeleOp(name="StingerTele", group="Swarm")
@Config

//@Disabled
public class StingerTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor backLift = null;
    private DcMotor arm = null;
    private DcMotor climbRight = null;
    private DcMotor climbLeft = null;
    private CRServo intake = null;
    private TouchSensor backLiftLimit = null;
    private TouchSensor armLimit = null;


    //TODO Decide names for and declare extra motors. (Top intake, bottom intake, lift)
    //TODO Decide names for and declare servos.


    public static double driveSpeed = 1.0;

    public static double liftSpeed = 1.0;
    public static double climbSpeed = 1.0;


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
        backLift = hardwareMap.get(DcMotor.class, "backLift");
        arm = hardwareMap.get(DcMotor.class, "arm");
        climbRight = hardwareMap.get(DcMotor.class, "climbRight");
        climbLeft = hardwareMap.get(DcMotor.class, "climbLeft");
        intake = hardwareMap.get(CRServo.class, "intake");
        backLiftLimit = hardwareMap.get(TouchSensor.class, "backLiftLimit");
        armLimit = hardwareMap.get(TouchSensor.class, "armLimit");


        //TODO initilize new motors that were added

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        backLift.setDirection(DcMotor.Direction.REVERSE); // slide
        arm.setDirection(DcMotor.Direction.FORWARD);
        climbRight.setDirection(DcMotor.Direction.FORWARD);
        climbLeft.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(CRServo.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        backLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





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
        double backLiftPower;
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

        double armInput = gamepad2.right_stick_y;
        if (Math.abs(armInput) < DEADZONE) {
            armInput = 0;
        }
        double armCubed = armInput * armInput * armInput;

        boolean climb = gamepad2.right_bumper;
        if (climb){
            climbPower=1;
        }else {
            climbPower=0;
        }

        boolean climbReverse = gamepad2.left_bumper;
        if (climbReverse){
            climbPower = -1;
        }
        boolean intakeInput = gamepad2.a;
        if (intakeInput){
            intakePower=1;
        }else {
            intakePower=0;
        }

        boolean intakeInputR = gamepad2.b;
        if (intakeInputR){
            intakePower = -1;
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
        backLiftPower = Range.clip(backLiftCubed, -1.0, 1.0);
        armPower = Range.clip(armCubed, -1.0, 1.0);

        if ( gamepad2.dpad_up) {
            if (arm.getCurrentPosition() < -3661) {
                arm.setPower(liftSpeed);
            } else if (arm.getCurrentPosition() > -3659) {
                arm.setPower(-liftSpeed);
            } else {
                arm.setPower(0);
            }
            if (backLift.getCurrentPosition() < 2575 & arm.getCurrentPosition() < -1661) {
                backLift.setPower(liftSpeed);
            } else {
                backLift.setPower(0);
            }

        } else if(gamepad2.dpad_down){
            if (arm.getCurrentPosition() < 0) {
                arm.setPower(liftSpeed);
            } else if (arm.getCurrentPosition() > 0) {
                arm.setPower(-liftSpeed);
            } else {
                arm.setPower(0);
            }
            if (backLift.getCurrentPosition() < 0 ) {
                backLift.setPower(liftSpeed);
            } else {
                backLift.setPower(0);
            }
        } else{

            arm.setPower(armPower * liftSpeed);
            if (backLiftPower < 0 && backLiftLimit.isPressed()) {
                backLift.setPower(0);
            } else {
                backLift.setPower(backLiftPower * liftSpeed);
            }
        }

        // Send calculated power to wheels

        leftRear.setPower(leftRearPower * driveSpeed);
        rightRear.setPower(rightRearPower * driveSpeed);
        leftFront.setPower(leftFrontPower * driveSpeed);
        rightFront.setPower(rightFrontPower * driveSpeed);
        climbRight.setPower(climbPower);
        climbLeft.setPower(-climbPower);
        intake.setPower(intakePower);



//        if (sensorDistance.getDistance(DistanceUnit.MM) < 5 && scoreCon && leftFrontPower > 0){
//            leftRear.setPower(0);
//            rightRear.setPower(0);
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//        }



//        if (backLiftPower < 0 && backLiftLimit.isPressed()) {
//            backLift.setPower(0);
//        } else {
//            backLift.setPower(backLiftPower * liftSpeed);
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
        telemetry.addData("lift", arm.getCurrentPosition());
        telemetry.addData("Slide", backLift.getCurrentPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

