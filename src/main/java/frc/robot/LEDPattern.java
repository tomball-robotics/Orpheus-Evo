package frc.robot;

public enum LEDPattern {
    // Format: (ID, PWM value, Description)
    RAINBOW_RAINBOW_PALETTE(1005, -0.99, "Fixed Palette: Rainbow (Rainbow Palette)"),
    RAINBOW_PARTY_PALETTE(1015, -0.97, "Fixed Palette: Rainbow (Party Palette)"),
    RAINBOW_OCEAN_PALETTE(1025, -0.95, "Fixed Palette: Rainbow (Ocean Palette)"),

    LARSON_SCANNER_RED(1325, -0.35, "Fixed Palette: Larson Scanner (Red)"),

    HEARTBEAT_SLOW(1515, 0.03, "Color 1 Pattern: Heartbeat Slow"),
    HEARTBEAT_MEDIUM(1525, 0.05, "Color 1 Pattern: Heartbeat Medium"),
    HEARTBEAT_FAST(1535, 0.07, "Color 1 Pattern: Heartbeat Fast"),

    COLOR_WAVES_COLOR_1_AND_2(1765, 0.53, "Color 1 and 2 Pattern: Color Waves"),

    SOLID_BLUE(1935, 0.87, "Solid Colors: Blue"),
    SOLID_BLUE_VIOLET(1945, 0.89, "Solid Colors: Blue Violet"),
    SOLID_VIOLET(1955, 0.91, "Solid Colors: Violet");

    private final int id;
    private final double value;
    private final String description;

    LEDPattern(int id, double value, String description) {
        this.id = id;
        this.value = value;
        this.description = description;
    }

    public int getId() {
        return id;
    }

    public double getValue() {
        return value;
    }

    public String getDescription() {
        return description;
    }

    public static LEDPattern fromValue(double pwmValue) {
        for (LEDPattern pattern : values()) {
            if (Math.abs(pattern.value - pwmValue) < 1e-6) {
                return pattern;
            }
        }
        return null; // or throw an exception if preferred
    }
}
