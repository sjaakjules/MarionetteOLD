using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using Microsoft.Xna.Framework;
using MathNet.Numerics.LinearAlgebra;

namespace Marionette
{
    public class Trajectory : Server
    {
        private List<double[,]> quinticParameters;
        private readonly Quaternion finalOrientation;
        public double AVE_SPEED = 200; // Average velocity used to calculate trajectories (mm/s)
        private TimeSpan trajectoryTime;
        private bool isActive;
        private Stopwatch elapsedTime;

        /// <summary>
        /// Overloaded constructor for various poses
        /// 
        /// Set up variables, quaternion for slerp and time from cardinal distance
        /// Populate quintinParameters with x y z quintic curves where each entry in the list is the next point
        /// </summary>
        public Trajectory(double[] pose)
        {
            
            finalOrientation = MakeQuaternion(pose);
            double distance = Math.Sqrt(Math.Pow(pose[0] - PosCard[0], 2) + Math.Pow(pose[1] - PosCard[1], 2) + Math.Pow(pose[2] - PosCard[2], 2));
            trajectoryTime = new TimeSpan(0, 0, 0, 0, Convert.ToInt32(1000 * 1.2 * (distance / AVE_SPEED)));

        }

        /// <summary>
        /// Controls when the trajectory begins moving
        /// </summary>
        public Boolean IsActive
        {
            get
            {
                return isActive;
            }
            set
            {
                if (!isActive & value)
                {
                    isActive = value;
                    elapsedTime.Start();
                }
                else if (isActive & !value)
                {
                    isActive = value;
                    elapsedTime.Stop();
                }
            }
        }

        /// <summary>
        /// Gets the Position at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefPos()
        {
            if (elapsedTime.ElapsedMilliseconds > trajectoryTime.Milliseconds)
            {
                return -1;
            }
            else
                return quinticParameters[0][0, 0] +
                    quinticParameters[0][1, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 1) +
                    quinticParameters[0][2, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 2) +
                    quinticParameters[0][3, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 3) + 
                    quinticParameters[0][4, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 4) + 
                    quinticParameters[0][5, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 5);
        }

        /// <summary>
        /// Gets the velocity at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefVel()
        {
            if (elapsedTime.ElapsedMilliseconds > trajectoryTime.Milliseconds)
            {
                return -1;
            }
            else
                return quinticParameters[0][1, 0] +
                    2 * quinticParameters[0][2, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 1) + 
                    3 * quinticParameters[0][3, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 2) + 
                    4 * quinticParameters[0][4, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 3) + 
                    5 * quinticParameters[0][5, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 4);
        }

        /// <summary>
        /// Gets the acceleration at a specified time, returns -1 if out of time boundaries
        /// </summary>
        public double RefAcc()
        {
            if (elapsedTime.ElapsedMilliseconds > trajectoryTime.Milliseconds)
            {
                return -1;
            }
            else
                return 2 * quinticParameters[0][2, 0] + 
                    6 * quinticParameters[0][3, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 1) + 
                    12 * quinticParameters[0][4, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 2) + 
                    20 * quinticParameters[0][5, 0] * Math.Pow(elapsedTime.ElapsedMilliseconds, 3);
        }

        /// <summary>
        /// Given a pose it calculates the quintic coefficients conserving the current position and velocity
        /// It assumes zero initial time, zero final velocity and when half the time has elapsed it is hafway  with the average velocity
        /// </summary>
        private void FindQuintic(double[] poses)
        {
            Matrix<double> tempQuinticParam = Matrix<double>.Build.Dense(6, 3);
            for (int i = 0; i < 3; i++)
            {                
                tempQuinticParam.SetColumn(i, FindCurve(0, trajectoryTime.Milliseconds, PosCard[i], poses[i], VelCard[i], 0)); // magic numbers are zero start time and zero final velocity
            }
            quinticParameters.Add(tempQuinticParam.ToArray());
        }

        /// <summary>
        /// Finds the coefficients to describe a quintic path using start and final time, position and velocity
        /// It assumes when half the time has elapsed it is hafway and with the average velocity
        /// </summary>
        private Vector<double> FindCurve(double t0, double tf, double x0, double xf, double v0, double vf)
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
                                                                                {AVE_SPEED}});                   // mid velocity 
            Matrix<Double> X = A.Solve(Y);
            return X.Column(0);
        }

        
    }
}
