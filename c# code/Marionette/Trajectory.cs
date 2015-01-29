using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using MathNet.Numerics.LinearAlgebra;

namespace Marionette
{
    public class Trajectory : Server
    {
        private List<double[,]> quinticParameters;
        private readonly Quaternion finalOrientation;
        public double AVE_SPEED = 200; // Average velocity used to calculate trajectories (mm/s)
        private double trajectoryTime;

        /// <summary>
        /// Overloaded constructor for various poses
        /// 
        /// Set up variables, quaternion for slerp and time from cardinal distance
        /// Populate quintinParameters with x y z quintic curves where each entry in the list is the next point
        /// </summary>
        public Trajectory(double[] poses)
        {
            
            finalOrientation = MakeQuaternion(poses);
            double distance = Math.Sqrt(Math.Pow(poses[0] - PosCard[0], 2) + Math.Pow(poses[1] - PosCard[1], 2) + Math.Pow(poses[2] - PosCard[2], 2));
            trajectoryTime = distance / AVE_SPEED;

        }

        /// <summary>
        /// Gets the Position at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefPos()
        {
            throw new System.NotImplementedException();
        }

        /// <summary>
        /// Gets the velocity at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefVel()
        {
            throw new System.NotImplementedException();
        }

        /// <summary>
        /// Gets the acceleration at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefAcc()
        {
            throw new System.NotImplementedException();
        }

        /// <summary>
        /// Finds the coefficients to describe a quintic path from current position, velocity and acceleration
        /// </summary>
        private void FindQuintic(double[] poses)
        {
            double t0 = PosCard[1];
            
        }

        private void FindCurve(double t0, double tf, double x0, double xf, double v0, double vf)
        {
            double tm = (tf - t0) / 2;
            Matrix<Double> A = Matrix<Double>.Build.DenseOfArray(new double[,] {{1,  t0  ,Math.Pow(t0,2),Math.Pow(t0,3)  , Math.Pow(t0,4)  , Math.Pow(t0,5)  },  // start position
                                                                                {0,  1   , 2*t0         ,3*Math.Pow(t0,2), 4*Math.Pow(t0,3), 5*Math.Pow(t0,4)},  // start velocity 
                                                                                {1,  tm  ,Math.Pow(tm,2),Math.Pow(tm,3)  ,  Math.Pow(tm,4) , Math.Pow(tm,5)  },  // mid position 
                                                                                {1,  tf  ,Math.Pow(tf,2),Math.Pow(tf,3)  ,  Math.Pow(tf,4) , Math.Pow(tf,5)  },  // final position
                                                                                {0,  1   , 2*tf         ,3*Math.Pow(tf,2), 4*Math.Pow(tf,3), 5*Math.Pow(tf,4)},  // final velocity
                                                                                {0,  1   , 2*tm         ,3*Math.Pow(tm,2), 4*Math.Pow(tm,3), 5*Math.Pow(tm,4)}}); // mid velocity 
            Matrix<Double> Y = Matrix<Double>.Build.DenseOfArray(new double[,] {{x0},                           // start position
                                                                                {v0},                           // start velocity 
                                                                                {x0+(xf-x0)/2},                 // mid position (half way)
                                                                                {xf},                           // final position
                                                                                {vf},                           // final velocity
                                                                                {(xf-x0)/(0.6*(tf-t0))}});      // mid velocity (60% of average velocity)
        }

        
    }
}
