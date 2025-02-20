// -- Explicatii pentru cod --
// Toate Motoarele si Servo-urile sunt declarate inainte de mainInit() si se declara prin metodele urmatoare:
//  1. Motoarele sunt declarate prin -- DcMotor nume_motor --
//  2. Servo-urile sunt declarate prin -- Servo nume_servo --
//  3. Servo-urile de rotatie continua sunt declarate prin -- CRServo nume_crservo --
//  4. Senzorii sunt declarati -- TouchSensor nume_tsensor, ColorSensor nume_csensor, DistanceSensor nume_dsensor --
//
// Inauntru lui mainInit() motoarele se initializeaza prin metodele urmatoare:
//  1. Motoarele sunt initializate prin -- nume_motor = hardwareMap.dcMotor.get("nume_motor_in_configuratie");
//  2. Servo-urile sunt initializate prin -- nume_servo = hardwareMap.servo.get("nume_servo_in_configuratie");
//  3. Servo-urile pentru rotatie sunt initializate prin -- hardwareMap.crservo.get("nume_crservo_in_configuratie");
//  4. Senzorii sunt initializati prin -- nume/tip_senor = hardwareMap.get(Tipul(Touch, Color, Direction)Sensor.class, "nume_senzor_in_configuratie");
//
// Daca motoarele sunt montate paralel, de obicei pentru motoarele din dreapta trebuie dat un nume_motor.setDirection(DcMotorSimple.Direction.REVERSE);
// In timpul initializarii este recomandat ca toate motoarele sa aiba un apel de nume_motor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehaviour.BRAKE); pentru a pune frana la motoare
// Pentru a activa encoder-ul unui motor putem apela nume_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// NOTE: Pentru motoarele care ruleaza pe encodere recomand sa apelam nume_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); in timpul initializarii
//
// Pentru "debugging" in mainLoop() pot fi folosite telemetry-uri ca sa dam display la anumite lucruri de care avem nevoie
// Exemplu: pentru pozitia unui motor (cu encoder): telemetry.addData("nume_motor_pos: ", nume_motor.getCurrentPosition());
// NOTE: codul pentru telemetry se sfarseste mereu cu un telemetry.update();
//
// Pentru a misca motoarele sau servourile putem scrie urmatoarele linii de cod
//
// nume_motor.setPower(1); >> inauntru este un parametru de tip double, putem pune -1 ca sa rulam motorul in directia opusa
// nume_servo.setPosition(1); >> pozitiile pot fi intre 0 si 1 (de exemplu 0.5)
// nume_crservo.setPower(1); >> pentru servo-urile de rotatie continua
//
//
// TeleOp-ul se programeaza prin thread-uri
// Pentru a crea un thread scriem codul urmator in mainInit();
// registerThread(new TaskThread(() -> {
//      (De exemplu)
//      if(gamepad1.a) nume_servo.setPosition(1);
// }));
//
//
// Pentru a da upload la cod in robot putem folosi ADB-WIFI ca sa dam upload la cod prin reteaua de internet a robotului
// SAU
// Se poate realiza prin a conecta un cablu de tip USB-C ca sa dam upload la cod
// Butonul pentru upload este cel de Run TeamCode (butonul cu o sageata verde si una alba) / SHIFT-F10

package org.firstinspires.ftc.teamcode;




import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

// @Disabled
@TeleOp

public class Manetutza extends ThreadOpMode {

    DcMotor fdr, sdr, fst, sst; // sasiu

    DcMotor gdr, gst; // glisiera

    Servo bratSt, bratDr, cleste;

//    public static double gstP = 0.02, gdrP = 0.02, gstI = 0.03, gdrI = 0.03, gstD = 0.0001, gdrD = 0.0001;
//    public static double gstF = 0.01, gdrF = 0.01;
//    private PIDController gdrC, gstC;
//    private final double ticks2deg = 384.5 / 360.0;
//    private int setPoint = 0;
//    double saturation = 0.5;
//    private ElapsedTime timp = new ElapsedTime();
//    int trc = 0;

    @Override
    public void mainInit() {

        fdr = hardwareMap.dcMotor.get("fdr");
        fdr.setDirection(DcMotorSimple.Direction.REVERSE);
        fdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sdr = hardwareMap.dcMotor.get("sdr");
        sdr.setDirection(DcMotorSimple.Direction.REVERSE);
        sdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fst = hardwareMap.dcMotor.get("fst");
        fst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sst = hardwareMap.dcMotor.get("sst");
        sst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gdr = hardwareMap.dcMotor.get("gdr");
        gdr.setDirection(DcMotorSimple.Direction.REVERSE);
        gdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gst = hardwareMap.dcMotor.get("gst");
        gst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bratSt = hardwareMap.servo.get("bratSt");
        bratSt.setPosition(0);

        bratDr = hardwareMap.servo.get("bratDr");
        bratDr.setDirection(Servo.Direction.REVERSE);
        bratDr.setPosition(0);

        cleste = hardwareMap.servo.get("cleste");
        cleste.setPosition(0.4);



        registerThread(new TaskThread(() -> {
            // sasiu
            sst.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            sdr.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            fst.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            fdr.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));

        }));

        registerThread(new TaskThread(() -> {
            if(gamepad2.dpad_up) {
                nivel1();
            }

            if(gamepad2.dpad_down) {
                coborare();
            }

            if(gamepad2.dpad_left) {
                nivel2();
            }

            if(gamepad2.dpad_right) {
                tractiune();
            }
        }));


        // gamepad 2
        registerThread(new TaskThread(() -> {

            if(gamepad2.left_bumper) {
                bratSt.setPosition(0.69);
                bratDr.setPosition(0.69);
            }

            if(gamepad2.right_bumper) {
                bratSt.setPosition(0.37);
                bratDr.setPosition(0.37);
            }

            if(gamepad2.y) {
                bratSt.setPosition(0.50);
                bratDr.setPosition(0.50);
            }

            if(gamepad2.right_trigger >= 0.5) {
                bratSt.setPosition(0);
                bratDr.setPosition(0);
            }

            if(gamepad2.b) {
                cleste.setPosition(1);
            }

            if(gamepad2.a) {
                cleste.setPosition(0.4);
            }

        }));

    }
    @Override
    public void mainLoop() {
        telemetry.addData("gdr_pos: ", gdr.getCurrentPosition());
        telemetry.addData("gst_pos: ", gst.getCurrentPosition());
        telemetry.addData("bratSt_pos: ", bratSt.getPosition());
        telemetry.addData("bratDr_pos: ", bratDr.getPosition());
        telemetry.addData("cleste_pos: ", cleste.getPosition());
        telemetry.update();
    }

    /****************************************************************************************
     ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
     ****************************************************************************************/

//    public void nivel(int punct) {
//        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brat2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brat.setTargetPosition(punct);
//        brat2.setTargetPosition(punct);
//        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        brat2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (brat.isBusy() || brat2.isBusy()) {
//            brat.setPower(0.1);
//            brat2.setPower(0.1);
//        }
//    }



    public void nivel1() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(1200);
        gst.setTargetPosition(1200);
        gdr.setPower(0.5);
        gst.setPower(0.5);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() && gst.isBusy()) {
            sleep(1);
        }
        gdr.setPower(0.01);
        gst.setPower(0.01);
    }

    public void nivel2() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(3300);
        gst.setTargetPosition(3300);
        gdr.setPower(0.5);
        gst.setPower(0.5);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() && gst.isBusy()) {
            sleep(1);
        }
        gdr.setPower(0.01);
        gst.setPower(0.01);
    }

    public void tractiune() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(1360);
        gst.setTargetPosition(1360);
        gdr.setPower(0.5);
        gst.setPower(0.5);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() && gst.isBusy()) {
            sleep(1);
        }
        gdr.setPower(0.01);
        gst.setPower(0.01);
    }


    public void coborare() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(0);
        gst.setTargetPosition(0);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() || gst.isBusy()) {
            gdr.setPower(0.6);
            gst.setPower(0.6);
        }
    }

//
//    double getBatteryVoltage(){
//        double result = Double.POSITIVE_INFINITY;
//        for(VoltageSensor s: hardwareMap.voltageSensor){
//            double voltage = s.getVoltage();
//            if(voltage > 0){
//                result = Math.min(result, voltage);
//            }
//        }
//        return result;
//    }
//
//    public void runGlisiera(){
//
//        gdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        gst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        gdrC.setPID(gdrP, gdrI, gdrD);
//        gstC.setPID(gstP, gstI, gstD);
//
//        int gdrPos = gdr.getCurrentPosition();
//        int gstPos = gst.getCurrentPosition();
//
//        double gdrPID = gdrC.calculate(gdrPos, setPoint);
//        double gstPID = gstC.calculate(gstPos, setPoint);
//
//        double gdrFF = Math.cos(Math.toRadians(setPoint / ticks2deg)) * gdrF;
//        double gstFF = Math.cos(Math.toRadians(setPoint / ticks2deg)) * gstF;
//
//        double gdrPower = gdrPID + gdrFF;
//        double gstPower = gstPID + gstFF;
//
//        if(gdrPower >= saturation)
//            gdrPower = saturation;
//        if(gdrPower <= -saturation)
//            gdrPower = -saturation;
//        if(gdr.getCurrentPosition() <= 300 && !touchDR.isPressed() && setPoint == 0 && !gamepad2.a)
//            gdrPower = -0.3;
//
//        if(gstPower >= saturation)
//            gstPower = saturation;
//        if(gstPower <= -saturation)
//            gstPower = -saturation;
//        if(gst.getCurrentPosition() <= 300 && !touchST.isPressed() && setPoint == 0 && !gamepad2.a)
//            gstPower = -0.3;
//
//        gdr.setPower(gdrPower);
//        gst.setPower(gstPower);
//        if(touchDR.isPressed()) {
//            gdr.setPower(0);
//            gdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//        if(touchST.isPressed()) {
//            gst.setPower(0);
//            gst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//        if(gst.getCurrentPosition() >= 300 && gdr.getCurrentPosition() >= 300 && setPoint <= 1500)
//        if(setPoint == 0 || setPoint > 1600)
//
//
//        telemetry.addData("target ", setPoint);
//        telemetry.addData("gstPos ", gstPos);
//        telemetry.addData("gdrPos ", gdrPos);
//        telemetry.addData("gstPower ", gstPower);
//        telemetry.addData("gdrPower ", gdrPower);
//        telemetry.update();
//    }
//
//
}

