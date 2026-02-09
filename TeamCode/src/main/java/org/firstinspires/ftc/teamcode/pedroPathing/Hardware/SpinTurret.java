package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class SpinTurret {
    private CRServo SpinTourelle;
    private double powerarrettourelle = 0.0;
    private double powertournertourelle =1.0;
    public IMU imuTourelle;
    private boolean turretZeroDone = false;
    private long imuReadySince = 0;
    double erreur;

    private BNO055IMU boschImu;

    private enum spintourelleetat {
        IDLE,
        CentrageZeroTourelle, // PositionCentrale
        TournerJoystick,
        TournerAutoCamera,
        TournerAutoBlueSansCamera,
        TournerAutoRedSansCamera,

    }
    private spintourelleetat Spintourelleetat = spintourelleetat.IDLE;

    public void init(@NonNull HardwareMap hwMap) {
        SpinTourelle = hwMap.get(CRServo.class, "SpinTourelle");
        SpinTourelle.setDirection(CRServo.Direction.REVERSE);
        //SpinTourelle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Mettre le moteur en mode BRAKE
        //SpinTourelle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //SpinTourelle.setPower(0);

        imuTourelle = hwMap.get(IMU.class, "imuTourelle");
        IMU.Parameters parameters = new IMU.Parameters(
                new Rev9AxisImuOrientationOnRobot(
                        Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP,
                        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.LEFT)
        );
        imuTourelle.initialize(parameters);
        imuTourelle.resetYaw();




    }
    public void update() {

        switch (Spintourelleetat) {
            case IDLE:
                SpinTourelle.setPower(powerarrettourelle);
                break;

            case CentrageZeroTourelle:
                if (lectureangletourelle() > 2) {
                    SpinTourelle.setPower(-powertournertourelle);
                }
                if (lectureangletourelle() < -2){
                    SpinTourelle.setPower(powertournertourelle);
                }
                if (lectureangletourelle() <= 2 && lectureangletourelle() >= -2){
                    SpinTourelle.setPower(powerarrettourelle);
                }
                break;
            case TournerJoystick:
                //rotationtourelle();
                break;


        }
    }
    // Méthode pour lecture de l'angle gyro de la tourelle et pour faire tourner la tourelle avec un angle max
    public double lectureangletourelle() {
        double angletourelle = imuTourelle.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angletourelle; // en degrés;
    }
    public void rotationtourelle(double power) {
        // Limitation de la puissance maximale
        double maxPower = 0.13; // valeur maximale autorisée
        power = Math.max(-maxPower, Math.min(power, maxPower)); // clamp entre -1 et +1

        double angletourelle = lectureangletourelle();
        if ((angletourelle >= 62 && power < 0) || (angletourelle <= -62 && power > 0)) {
            SpinTourelle.setPower(0);
        } else {
            SpinTourelle.setPower(power);
        }
    }

    public void allerVersAngle(double angleCible) {
        double angleActuel = lectureangletourelle();
        erreur = angleActuel - angleCible;

        double absErr = Math.abs(erreur);
        double power;

        // 1) Fenêtre d'arrêt : on stoppe complètement
        if (absErr < 5.0) {
            power = 0.0;  // STOP
        }
        // 2) Zone proche : on bouge lentement mais avec un minimum
        else if (absErr < 10.0) {
            double minPower = 0.1; //
            power = Math.signum(erreur) * minPower;
        }
        // 3) Zone lointaine : proportionnel normal
        else {
            double kP = 0.05;
            power = kP * erreur;

            power = Math.max(-0.11, Math.min(power, 0.11));
        }

        SpinTourelle.setPower(power);
    }



    // Connaitre l'angle de la tourelle
    public boolean isAtAngle(double angleCible) {
        return Math.abs(lectureangletourelle() - angleCible) < 5.0;
    }

    public void stopTourelle() {
        SpinTourelle.setPower(0);
    }// blocage servo }

    public double geterreur(){
        return erreur;
    }

    public void resetImuToutelle(){
        imuTourelle.resetYaw();
    }


    public void tournerAvecTx(double tx) {

        double kP = 0.03;       // un peu plus fort
        double maxPower = 0.30; // vitesse max
        double minPower = 0.12; // seuil mécanique réaliste

        double power = kP * tx;

        // Appliquer une puissance minimale si on est encore loin du centre
        if (Math.abs(tx) > 2.0 && Math.abs(power) < minPower) {
            power = Math.signum(tx) * minPower;
        }

        // Limitation
        power = Math.max(-maxPower, Math.min(power, maxPower));

        // Zone morte finale
        if (Math.abs(tx) < 1.0) {
            power = 0;
        }

        SpinTourelle.setPower(power);
    }

    public void tournerAvecTxoffset(double tx, double offset) {

        double kP = 0.03;          // gain proportionnel
        double maxPower = 0.19;    // palier 4 : > 15°
        double power5 = 0.179;     // palier 1 : 0–5°
        double power10 = 0.183;     // palier 2 : 5–10°
        double power15 = 0.185;     // palier 3 : 10–15°


        // Erreur corrigée selon l’alliance
        double erreur = tx - offset;
        double absErr = Math.abs(erreur);

        double power;

        // -----------------------------
        // PALIER 1 : zone morte (< 1°)
        // -----------------------------
        if (absErr < 1.0) {
            power = 0;
        }

        // -----------------------------
        // PALIER 2 : 1° à 5°
        // puissance minimale
        // -----------------------------
        else if (absErr < 5.0) {
            power = Math.signum(erreur) * power5;
        }

        // -----------------------------
        // PALIER 3 : 5° à 10°
        // deuxième palier
        // -----------------------------
        else if (absErr < 10.0) {
            power = Math.signum(erreur) * power10;
        }

        // -----------------------------
        // PALIER 4 : 10° à 15°
        // troisième palier
        // -----------------------------
        else if (absErr < 15.0) {
            power = Math.signum(erreur) * power15;
        }

        // -----------------------------
        // PALIER 5 : > 15°
        // puissance max
        // -----------------------------
        else {
            power = Math.signum(erreur) * maxPower;
        }

        // Limitation finale
        power = Math.max(-maxPower, Math.min(power, maxPower));

        SpinTourelle.setPower(power);
    }

    public boolean isAtTx(double tx, double TxCible) {
        return Math.abs(tx - TxCible) < 1.0;
    }


}







