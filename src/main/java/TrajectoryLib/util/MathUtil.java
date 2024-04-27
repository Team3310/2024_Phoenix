package TrajectoryLib.util;

public class MathUtil {
    
    /**
     * clamps the value of a number to between a min and max
     * @param a value to be clamped
     * @param min min value
     * @param max max value
     * @return the clamped value
     */
    public static double clamp(double a, double min, double max){
        if(a>max){
            return max;
        }

        if(a<min) {
            return min;
        }

        return a;
    }

    /**
   * Returns modulus of input.
   *
   * @param input Input value to wrap.
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   * @return The wrapped value.
   */
  public static double inputModulus(double input, double minimumInput, double maximumInput) {
    double modulus = maximumInput - minimumInput;

    // Wrap input if it's above the maximum input
    int numMax = (int) ((input - minimumInput) / modulus);
    input -= numMax * modulus;

    // Wrap input if it's below the minimum input
    int numMin = (int) ((input - maximumInput) / modulus);
    input -= numMin * modulus;

    return input;
  }
}
