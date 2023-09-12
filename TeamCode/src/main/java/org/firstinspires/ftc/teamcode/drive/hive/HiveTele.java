package org.firstinspires.ftc.teamcode.drive.hive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="SwarmTele", group="Iterative Opmode")
@Config

//@Disabled
public class HiveTele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor RightFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightBack = null;


    public static double DriveSpeed = 0.6;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.FORWARD);

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
        double LeftBackPower;
        double RightBackPower;
        double LeftFrontPower;
        double RightFrontPower;


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        final double DEADZONE = 0.1;

        double Drive = -gamepad1.left_stick_y;
        if (Math.abs(Drive) < DEADZONE) {
            Drive = 0;
        }
        double DriveCubed = Drive * Drive * Drive;

        double Strafe = gamepad1.left_stick_x;
        if (Math.abs(Strafe) < DEADZONE) {
            Strafe = 0;
        }
        double StrafeCubed = Strafe * Strafe * Strafe;

        double Spin = gamepad1.right_stick_x;
        if (Math.abs(Spin) < DEADZONE) {
            Spin = 0;
        }
        double SpinCubed = Spin * Spin * Spin;


        LeftBackPower = Range.clip(DriveCubed + SpinCubed - StrafeCubed, -1.0, 1.0);
        RightBackPower = Range.clip(DriveCubed - SpinCubed + StrafeCubed, -1.0, 1.0);
        LeftFrontPower = Range.clip(DriveCubed + SpinCubed + StrafeCubed, -1.0, 1.0);
        RightFrontPower = Range.clip(DriveCubed - SpinCubed - StrafeCubed, -1.0, 1.0);

        // Send calculated power to wheels
        LeftBack.setPower(LeftBackPower * DriveSpeed);
        RightBack.setPower(RightBackPower * DriveSpeed);
        LeftFront.setPower(LeftFrontPower * DriveSpeed);
        RightFront.setPower(RightFrontPower * DriveSpeed);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", LeftBackPower, RightBackPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

