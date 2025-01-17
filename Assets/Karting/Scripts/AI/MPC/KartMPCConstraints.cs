using CenterSpace.NMath.Core;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KartGame.AI.MPC
{

    /**
    * Abstract/interface representations of Costs that would be part of the MPC algorithm
    * The implemented functions add the constraints to a Problem instance and return if they are satisfied for a given trajectory
     **/
    public interface KartMPCConstraints
    {
        void AddConstraints(NonlinearProgrammingProblem problem);
        bool isSatisfied(DoubleVector x);
    }

    public abstract class CoupledKartMPCConstraints: KartMPCConstraints
    {
        public DoubleVector other;
        public int otherIdx;

        public abstract void AddConstraints(NonlinearProgrammingProblem problem);
        public abstract bool isSatisfied(DoubleVector x);
    }

    /**
     * Constraint to ensure the car is on track.
     * Player is at most some distance away from the nearest progress checkpoint defining the center line of the track
    **/
    public class OnTrackConstraint : KartMPCConstraints
    {
        List<Vector2> centerPoints;
        int initialIndex;
        int finalIndex;
        double maxDist;

        public OnTrackConstraint(List<Vector2> centerPoints, int initialIndex, int finalIndex, double maxDist)
        {
            this.centerPoints = centerPoints;
            this.initialIndex = initialIndex;
            this.finalIndex = finalIndex;
            this.maxDist = maxDist;
        }

        public void AddConstraints(NonlinearProgrammingProblem problem)
        {
            int T = problem.NumVariables / (KartMPC.xDim + KartMPC.uDim);
            for (int t = 1; t < T; t++)
            {
                int i = t;
                problem.AddNonlinearConstraint(new NonlinearConstraint(problem.NumVariables, (vec) =>
                {
                    double minDist = 1000 * 1000;
                    for (int j = initialIndex; j < finalIndex; j++)
                    {
                        minDist = Math.Min(Math.Pow(centerPoints[j % centerPoints.Count].x - vec[KartMPC.xIndex * T + (i)], 2) + Math.Pow(vec[KartMPC.zIndex * T + (i)] - centerPoints[j % centerPoints.Count].y, 2), minDist);
                    }
                    return maxDist*maxDist - minDist;
                }, ConstraintType.GreaterThanOrEqualTo));

            }
        }

        public bool isSatisfied(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            for (int t = 1; t < T; t++)
            {
                int i = t;
                double minDist = 1000 * 1000;
                for (int j = initialIndex; j < finalIndex; j++)
                {
                    minDist = Math.Min(Math.Pow(centerPoints[j % centerPoints.Count].x - x[KartMPC.xIndex * T + (i)], 2) + Math.Pow(x[KartMPC.zIndex * T + (i)] - centerPoints[j % centerPoints.Count].y, 2), minDist);
                }
                if (maxDist - minDist < 0)
                {
                    return false;
                }

            }
            return true;
        }
    }

    /**
     * Constraint limiting how close two players can get next to one antoher. 
     * THis is to ensure collision avoidance.
    **/
    public class CoupledDistanceConstraint : CoupledKartMPCConstraints
    {
        double minDist;
        public CoupledDistanceConstraint (double minDist, int otherIdx)
        {
            this.otherIdx = otherIdx;
            this.minDist = minDist;
        }
        public override void AddConstraints(NonlinearProgrammingProblem problem)
        {
            int T = problem.NumVariables / (KartMPC.xDim + KartMPC.uDim);
            for (int t = 1; t < T; t++)
            {
                int i = t;
                problem.AddNonlinearConstraint(new NonlinearConstraint(problem.NumVariables, (vec) => {
                double dist2 = Math.Pow(vec[KartMPC.xIndex * T + (i)] - other[KartMPC.xIndex * T + (i)], 2) + Math.Pow(vec[KartMPC.zIndex * T + (i)] - other[KartMPC.zIndex * T + (i)], 2);
                    return minDist * minDist - dist2;

                }, ConstraintType.GreaterThanOrEqualTo));
            }
        }

        public override bool isSatisfied(DoubleVector x)
        {
            int T = x.Length / (KartMPC.xDim + KartMPC.uDim);
            for (int t = 1; t < T; t++)
            {
                int i = t;
                double dist2 = Math.Pow(x[KartMPC.xIndex * T + (i)] - other[KartMPC.xIndex * T + (i)], 2) + Math.Pow(x[KartMPC.zIndex * T + (i)] - other[KartMPC.zIndex * T + (i)], 2);
                if (dist2 < minDist * minDist)
                    return false;
            }
            return true;
        }
    }
}