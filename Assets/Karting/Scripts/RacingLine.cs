using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.IO;

namespace KartGame.AI
{
    public class RacingLine {
        /**
         * Define functions related to tracking racing line (new addition)
         * We assume linear interpolation for discretization everywhere
        **/
        public List<Vector3> race_line;
        List<float> curvs;
        public float total_length = 0.0f;
        public RacingLine(List<Vector2> race_line_in)
        {
            race_line = new List<Vector3>();
            for (int i = 0;i<race_line_in.Count;i++){
                race_line.Add(new Vector3(race_line_in[i].x,0.279f,race_line_in[i].y));
            }
        }

        public RacingLine(String filename,bool ignore_first_line)
        {
            race_line = new List<Vector3>();
            curvs = new List<float>();
            StreamReader strReader = new StreamReader(filename);
            bool endOfFile = false;
            int line_no = 0;
            Debug.Log("Started reading race line");
            while(!endOfFile) {
                string line_string = strReader.ReadLine ();
                line_no++;
                if (ignore_first_line&&line_no<=1) continue;
                if (line_string == null){
                    endOfFile = true;
                    break;
                }
                var line_values = line_string.Split(';');
                race_line.Add(new Vector3(float.Parse(line_values[1]),float.Parse("0.279"),float.Parse(line_values[2])));
                if (race_line.Count>1)
                    total_length += Vector3.Distance(race_line[race_line.Count-2],race_line[race_line.Count-1]);
                curvs.Add(float.Parse(line_values[4]));
            }
            total_length += Vector3.Distance(race_line[race_line.Count-1],race_line[0]);
            Debug.Log("Length of racing line : " + race_line.Count);
            UnityEngine.Debug.Log("Hahahahaha");
        }

        public float getClosestDist(Vector3 point){
            int n = race_line.Count;
            if (n < 2)
            {
                Debug.LogError("At least two waypoints are required to define a line.");
                return -1;
            }

            float closestDistance = float.MaxValue;
            Vector3 closestPoint = Vector3.zero;
            float signedDist = 0.0f;
            for (int i = 0; i < n; i++)
            {
                Vector3 lineStart = race_line[i];
                Vector3 lineEnd = race_line[(i + 1)%n];

                Vector3 lineDirection = lineEnd - lineStart;
                float lineMagnitude = lineDirection.magnitude;
                Vector3 pointToLineStart = point - lineStart;

                float dotProduct = Vector3.Dot(pointToLineStart, lineDirection.normalized);
                float t = Mathf.Clamp01(dotProduct / lineMagnitude);

                Vector3 projection = lineStart + lineDirection.normalized * (t * lineMagnitude);

                // Check if the projected point is outside the line segment
                if (t <= 0f)
                {
                    projection = lineStart;
                }
                else if (t >= 1f)
                {
                    projection = lineEnd;
                }

                float distance = Vector3.Distance(point, projection);

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = projection;
                    var comp = Vector3.Cross(lineDirection.normalized, point - lineStart).y;
                    if (comp>0.0f) {
                        signedDist = closestDistance;
                    }
                    else {
                        signedDist = -closestDistance;
                    }
                }
            }
            return signedDist;
        }

        public float getClosestS(Vector3 point){
            int n = race_line.Count;
            if (n < 2)
            {
                Debug.LogError("At least two waypoints are required to define a line.");
                return -1;
            }

            float closestDistance = float.MaxValue;
            Vector3 closestPoint = Vector3.zero;
            float total_dist = 0.0f;
            float return_s = 0.0f;
            float signedDist = 0.0f;
            for (int i = 0; i < n; i++)
            {
                Vector3 lineStart = race_line[i];
                Vector3 lineEnd = race_line[(i + 1)%n];

                Vector3 lineDirection = lineEnd - lineStart;
                float lineMagnitude = lineDirection.magnitude;
                Vector3 pointToLineStart = point - lineStart;

                float dotProduct = Vector3.Dot(pointToLineStart, lineDirection.normalized);
                float t = Mathf.Clamp01(dotProduct / lineMagnitude);

                Vector3 projection = lineStart + lineDirection.normalized * (t * lineMagnitude);

                // Check if the projected point is outside the line segment
                if (t <= 0f)
                {
                    projection = lineStart;
                }
                else if (t >= 1f)
                {
                    projection = lineEnd;
                }

                float distance = Vector3.Distance(point, projection);

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = projection;
                    return_s = total_dist + Vector3.Distance(lineStart,projection);
                    var comp = Vector3.Cross(lineDirection.normalized, point - lineStart).y;
                    if (comp>0.0f) {
                        signedDist = closestDistance;
                    }
                    else {
                        signedDist = -closestDistance;
                    }
                }
                total_dist += Vector3.Distance(lineStart,lineEnd);
            }
            return return_s;
        }

        public Vector3 getClosestPoint(Vector3 point){
            int n = race_line.Count;
            if (n < 2)
            {
                Debug.LogError("At least two waypoints are required to define a line.");
                return Vector3.zero;
            }

            float closestDistance = float.MaxValue;
            Vector3 closestPoint = Vector3.zero;

            for (int i = 0; i < n-1; i++)
            {
                Vector3 lineStart = race_line[i];
                Vector3 lineEnd = race_line[i + 1];

                Vector3 lineDirection = lineEnd - lineStart;
                float lineMagnitude = lineDirection.magnitude;
                Vector3 pointToLineStart = point - lineStart;

                float dotProduct = Vector3.Dot(pointToLineStart, lineDirection.normalized);
                float t = Mathf.Clamp01(dotProduct / lineMagnitude);

                Vector3 projection = lineStart + lineDirection.normalized * (t * lineMagnitude);

                // Check if the projected point is outside the line segment
                if (t <= 0f)
                {
                    projection = lineStart;
                }
                else if (t >= 1f)
                {
                    projection = lineEnd;
                }

                float distance = Vector3.Distance(point, projection);

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = projection;
                }
            }
            return closestPoint;
        }
        
        public float getClosestAngle(Vector3 point){
            int n = race_line.Count;
            if (n < 2)
            {
                Debug.LogError("At least two waypoints are required to define a line.");
                return 0.0f;
            }

            float closestDistance = float.MaxValue;
            float closestAngle = 0.0f;
            Vector3 closestPoint = Vector3.zero;

            for (int i = 0; i < n; i++)
            {
                Vector3 lineStart = race_line[i];
                Vector3 lineEnd = race_line[(i + 1)%n];

                Vector3 lineDirection = lineEnd - lineStart;
                float lineMagnitude = lineDirection.magnitude;
                Vector3 pointToLineStart = point - lineStart;

                float dotProduct = Vector3.Dot(pointToLineStart, lineDirection.normalized);
                float t = Mathf.Clamp01(dotProduct / lineMagnitude);

                Vector3 projection = lineStart + lineDirection.normalized * (t * lineMagnitude);

                // Check if the projected point is outside the line segment
                if (t <= 0f)
                {
                    projection = lineStart;
                }
                else if (t >= 1f)
                {
                    projection = lineEnd;
                }

                float distance = Vector3.Distance(point, projection);

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = projection;
                    closestAngle = Mathf.Atan2(lineDirection.x,lineDirection.z);
                }
            }
            return closestAngle;
        }

        public float getClosestCurvature(Vector3 point){
            int n = race_line.Count;
            if (n < 2)
            {
                Debug.LogError("At least two waypoints are required to define a line.");
                return 0.0f;
            }

            float closestDistance = float.MaxValue;
            float closestCurvature = 0.0f;
            Vector3 closestPoint = Vector3.zero;

            for (int i = 0; i < n; i++)
            {
                Vector3 lineStart = race_line[i];
                Vector3 lineEnd = race_line[(i + 1)%n];

                Vector3 lineDirection = lineEnd - lineStart;
                float lineMagnitude = lineDirection.magnitude;
                Vector3 pointToLineStart = point - lineStart;

                float dotProduct = Vector3.Dot(pointToLineStart, lineDirection.normalized);
                float t = Mathf.Clamp01(dotProduct / lineMagnitude);

                Vector3 projection = lineStart + lineDirection.normalized * (t * lineMagnitude);

                // Check if the projected point is outside the line segment
                if (t <= 0f)
                {
                    projection = lineStart;
                }
                else if (t >= 1f)
                {
                    projection = lineEnd;
                }

                float distance = Vector3.Distance(point, projection);

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = projection;
                    closestCurvature = curvs[i];
                }
            }
            
            return closestCurvature;
        }
    }    
}
