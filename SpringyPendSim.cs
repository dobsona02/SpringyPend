using System;
using Godot;

public class SpringyPendSim
{
    private double Lo;  //Natural spring length
    private double g;   //Grav field constant
    private double m;   //Mass of pendulum bob
    private double k;   //Spring constant
    

    private int n;  //Number of Dif Eqs.
    private double[] x; //Array of states
    private double[] xA;  //Intermediate states for RK2
    private double[] xi;    //Intermediate states for RK4

    private double[][] f;    //Array of slopes
    private double[][] f4;   //Array of slopes for RK4

    // ------------------------------------------------------------
    //  Constructor
    // ------------------------------------------------------------
    public SpringyPendSim()
    {
        Lo = 0.9;
        g = 9.81;
        m = 1.4;
        k = 90.0;
        n = 6;
        x = new double[n];
        xA = new double[n];
        f = new double[2][];
        f[0] = new double[n];   //fA
        f[1] = new double[n];   //fB

        //RK4 Initialization
        xi = new double[n];
        f4 = new double[4][];
        f4[0] = new double[n];   //fA
        f4[1] = new double[n];   //fB
        f4[2] = new double[n];   //fC
        f4[3] = new double[n];   //fD

        x[0] = -0.7;    //Initial x
        x[1] = 0.0;    //Initial x dot
        x[2] = -0.2;    //Initial y
        x[3] = 0.0;    //Initial y dot
        x[4] = 0.5;    //Initial z
        x[5] = 0.9;    //Initial z dot

        
        
    }

    // ------------------------------------------------------------
    //  Step RK2: Completes RK2 step in solving equations of motion
    // ------------------------------------------------------------
    public void StepRK2(double time, double dt, Vector3 AncPos)
    {
        int i;

        RHSFuncPendulum(x, time, f[0], AncPos);
        for (i = 0; i < n; i++)
        {
            xA[i] = x[i] + f[0][i]*dt;
        }

        RHSFuncPendulum(xA, time+dt, f[1], AncPos);
        for (i = 0; i < n; i++)
        {
            x[i] = x[i] + 0.5*(f[0][i] + f[1][i])*dt;
        }
    }

    // ------------------------------------------------------------
    //  Step RK4: Completes RK4 step in solving equations of motion
    // ------------------------------------------------------------
    public void StepRK4(double time, double dt, Vector3 AncPos)
    {
        int i;

        //Calc fA
        RHSFuncPendulum(x, time, f4[0], AncPos);
        //Calc xA
        for (i = 0; i < n; i++)
        {
            xi[i] = x[i] + (0.5)*f4[0][i]*dt;
        }

        //Calc fB
        RHSFuncPendulum(xi, time+(0.5*dt), f4[1], AncPos);
        //Calc xB
        for(i = 0; i < n; i++)
        {
            xi[i] = x[i] + (0.5)*f4[1][i]*dt;
        }

        //Calc fC
        RHSFuncPendulum(xi, time+(0.5*dt), f4[2], AncPos);
        //Calc xC
        for(i = 0; i < n; i++)
        {
            xi[i] = x[i] + f4[2][i]*dt;
        }

        //Calc fD
        RHSFuncPendulum(xi, time+dt, f4[3], AncPos);
        //Calc xK+1
        for(i = 0; i < n; i++)
        {
            x[i] = x[i] + (1.0/6.0)*(f4[0][i] + (2.0)*f4[1][i] + (2.0)*f4[2][i] + f4[3][i])*dt;
        }
    }





    // ------------------------------------------------------------
    //  RHSFuncPendulum
    // ------------------------------------------------------------
    private void RHSFuncPendulum(double[] xx, double t, double[] ff, Vector3 AncPos)
    {
    double dx = xx[0] - AncPos.X;
    double dy = xx[2] - AncPos.Y;
    double dz = xx[4] - AncPos.Z;
    double L = Math.Sqrt((dx*dx) + (dy*dy) + (dz*dz));

    ff[0] = xx[1];
    ff[1] = -(k/m)*(L-Lo)*(dx/L);
    ff[2] = xx[3];
    ff[3] =-(k/m)*(L-Lo)*(dy/L) - g;
    ff[4] = xx[5];
    ff[5] = -(k/m)*(L-Lo)*(dz/L);
    }

    // ------------------------------------------------------------
    //  Calculate and return the energy
    // ------------------------------------------------------------
    public double[] CalcEnergy(Vector3 AncPos)
    {
        double dx = x[0] - AncPos.X;
        double dy = x[2] - AncPos.Y;
        double dz = x[4] - AncPos.Z;
        double L = Math.Sqrt((dx*dx) + (dy*dy) + (dz*dz));

        
        double PE, KE; //Potential and kinetic energy

        PE = (0.5)*(k)*(L-Lo)*(L-Lo) + (m)*(g)*(x[2]);
        KE = (0.5)*(m)*((x[1])*(x[1]) + (x[3])*(x[3]) + (x[5])*(x[5]));
       
       
        double[] Energy;
        Energy = new double[3];

        Energy[0] = PE;
        Energy[1] = KE;
        Energy[2] = PE + KE;
      

        return Energy;
    }

    // ------------------------------------------------------------
    //  Getters and setters
    // ------------------------------------------------------------
    public double XCoord
    {
        get
        {
            return(x[0]);
        }

        set
        {
            x[0] = value;
        }
    }

    public double YCoord
    {
        get
        {
            return(x[2]);
        }

        set
        {
            x[2] = value;
        }
    }

    public double ZCoord
    {
        get
        {
            return(x[4]);
        }

        set
        {
            x[4] = value;
        }
    }
}

