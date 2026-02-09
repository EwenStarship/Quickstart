package org.firstinspires.ftc.teamcode.pedroPathing.logique;

public class ShooterCalculator {

    /** Conteneur du résultat du calcul de tir */
    public static class ShooterResult {
        public boolean shouldShoot = false;
        /** Angle du volet (unité servo 0.0–1.0) */
        public double angle = 0.0;
        /** Vitesse du shooter (RPM) */
        public int rpm = 0;
    }

    /**
     * Calcule l’angle du volet et le RPM à partir de la distance caméra.
     * @param distanceCm distance au tag en centimètres (doit être > 0)
     * @return ShooterResult (shouldShoot=false si distance invalide)
     */
    public ShooterResult compute(double distanceCm) {
        ShooterResult r = new ShooterResult();

        if (distanceCm <= 0) {
            // distance invalide : on ne tire pas
            return r;
        }

        double distanceM = distanceCm / 100.0;

        // 1) RPM proportionnel (à ajuster selon ton robot si besoin)
        r.rpm = (int) Math.round(551.62 * distanceM + 3210.34);

        // 2) ANGLE (volet)
        // Base
        double angleBase = 0.30 * distanceM;

        // Correction 1 : +0.05 entre 1.10 m et 1.75 m
        double bonus = 0.0;
        if (distanceM >= 1.10 && distanceM <= 1.65) {
            bonus = 0.05;
        }

        // Correction 2 : réduction progressive entre 1.75 m et 2.00 m (jusqu’à -0.05)
        double reduction = 0.0;
        if (distanceM > 1.65) {
            // (distanceM - 1.75) varie de 0 à 0.25 -> interpolation linéaire vers 0.05
            reduction = 0.05;
        }
        if (distanceM > 2.00) {
            // (distanceM - 1.75) varie de 0 à 0.25 -> interpolation linéaire vers 0.05
            reduction = 0.00;
        }

        double angle = angleBase + bonus - reduction;


        // Clamp mécanique (sécurité servo)
        if (angle < 0.10) angle = 0.10;
        if (angle > 0.62) angle = 0.62;

        r.angle = angle;
        r.shouldShoot = true;
        return r;
    }
}