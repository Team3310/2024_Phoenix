package TrajectoryLib.splines;

import TrajectoryLib.util.MathUtil;

public class Spline {
    private double c0, c1, c2, c3, c4, c5;

    /**
     * creates quintic hermite spline coefficients from start to end points
     * @param start the starting point
     * @param end the ending point
     */
    public Spline(double p1, double v1, double p2, double v2){
        //calculate coefficients from the product of matrix math
        //TODO insert the math used
        this.c0 = p1;
        this.c1 = v1;
        //we assume acceleration is zero but kept the full expression
        this.c2 = 0.5*0.0; 
        //again the acceleration terms are zero but kept full expression
        this.c3 = -10*p1-6*v1-4*v2+10*p2  -1.5*0.0+0.5*0.0;
        this.c4 = 15*p1+8*v1+7*v2-15*p2   +1.5*0.0-0.0;
        this. c5 = -6*p1-3*v1-3*v2+6*p2   -0.5*0.0+0.5*0.0;
    }

    /**
     * get the position along the path
     * @param t 'time' value along path 0-1
     * @return the position at time t
     */
    public double getPosition(double t){
        t = MathUtil.clamp(t, 0, 1);
        return c0 + c1 * t + c2 * t*t + c3 * t*t*t + c4 * t*t*t*t + c5 * t*t*t*t*t; 
    }

    /**
     * get the velocity along the path
     * @param t 'time' value along path 0-1
     * @return the velocity at time t
     */
    public double getVelocity(double t){
        t = MathUtil.clamp(t, 0, 1);
        //the derivative of the position expression to get the velocity
        return c1 + 2.0 * c2 * t + 3.0 * c3 * t*t + 4.0 * c4 * t*t*t + 5.0 * c5 * t*t*t*t; 
    }

    public void printCoefficients(){
        System.out.println(String.format("c0: %.2f, c1: %.2f, c2: %.2f, c3: %.2f, c4: %.2f, c5: %.2f", c0, c1, c2, c3, c4, c5));
    }

    @Override
    public String toString(){
        return String.format("c0: %.2f, c1: %.2f, c2: %.2f, c3: %.2f, c4: %.2f, c5: %.2f", c0, c1, c2, c3, c4, c5);
    }
}
