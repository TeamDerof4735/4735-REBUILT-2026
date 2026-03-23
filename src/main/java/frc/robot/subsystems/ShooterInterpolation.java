package frc.robot;

public class ShooterLookup {

    public static class ShotPoint {
        public double distance;
        public double rpm;

        public ShotPoint(double distance, double rpm) {
            this.distance = distance;
            this.rpm = rpm;
        }
    }

    // ajustar
    private static final ShotPoint[] table = {
        new ShotPoint(1.5, 2000),
        new ShotPoint(2.0, 1980),
        new ShotPoint(2.5, 2100),
        new ShotPoint(3.0, 2200),
        new ShotPoint(3.5, 2500),
        new ShotPoint(4.0, 5000)
    };


    public static double getRPM(double d) {

        // si esta out of range
        if (d <= table[0].distance) {
            return table[0].rpm;
        }

        if (d >= table[table.length - 1].distance) {
            return table[table.length - 1].rpm;
        }

        for (int i = 0; i < table.length - 1; i++) {
            ShotPoint p1 = table[i];
            ShotPoint p2 = table[i + 1];

            if (d >= p1.distance && d <= p2.distance) {

                // Interpolacion
                double t = (d - p1.distance) / (p2.distance - p1.distance);

                return p1.rpm + t * (p2.rpm - p1.rpm);
            }
        }
        return 0;
    }
}