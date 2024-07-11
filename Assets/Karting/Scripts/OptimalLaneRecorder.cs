using KartGame.AI;
using KartGame.KartSystems;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;

public class OptimalLaneRecorder : MonoBehaviour
{
    
    public RacingEnvController m_envController;
    Vector3 lastClosestPoint;
    RacingLine race_line;
    // Start is called before the first frame update
    void Start()
    {
        int n = m_envController.Sections.Length;
        float mindist = 1000.0f;
        int minLane = -1;
        race_line = new RacingLine(m_envController.RacelineLoc,true);
        for (int i = 0; i < n; i++)
        {
            mindist = 1000.0f;
            minLane = -1;
            BoxCollider currLane = m_envController.Sections[i].Lane1;
            Vector3 center = (m_envController.Sections[i].Lane2.transform.position + m_envController.Sections[i].Lane3.transform.position)/2.0f;
            float currdist = Math.Abs(race_line.getClosestDist(currLane.transform.position));
            float currs = Math.Abs(race_line.getClosestS(currLane.transform.position));
            if (currdist<mindist){
                mindist = currdist;
                minLane = 1;
            }
            m_envController.Sections[i].diste.Add(race_line.getClosestDist(currLane.transform.position));
            m_envController.Sections[i].dists.Add(currs);
            // Debug.Log(1 + " " + currdist + " " + mindist);
            currLane = m_envController.Sections[i].Lane2;
            currdist = Math.Abs(race_line.getClosestDist(currLane.transform.position));
            currs = Math.Abs(race_line.getClosestS(currLane.transform.position));
            if (currdist<mindist){
                mindist = currdist;
                minLane = 2;
            }
            m_envController.Sections[i].diste.Add(race_line.getClosestDist(currLane.transform.position));
            m_envController.Sections[i].dists.Add(currs);
            // Debug.Log(2 + " " + currdist + " " + mindist);
            currLane = m_envController.Sections[i].Lane3;
            currdist = Math.Abs(race_line.getClosestDist(currLane.transform.position));
            currs = Math.Abs(race_line.getClosestS(currLane.transform.position));
            if (currdist<mindist){
                mindist = currdist;
                minLane = 3;
            }
            m_envController.Sections[i].diste.Add(race_line.getClosestDist(currLane.transform.position));
            m_envController.Sections[i].dists.Add(currs);
            // Debug.Log(3 + " " + currdist + " " + mindist);
            currLane = m_envController.Sections[i].Lane4;
            currdist = Math.Abs(race_line.getClosestDist(currLane.transform.position));
            currs = Math.Abs(race_line.getClosestS(currLane.transform.position));
            if (currdist<mindist){
                mindist = currdist;
                minLane = 4;
            }
            m_envController.Sections[i].diste.Add(race_line.getClosestDist(currLane.transform.position));
            m_envController.Sections[i].dists.Add(currs);
            // Debug.Log(4 + " " + currdist + " " + mindist);
            m_envController.Sections[i].optimalLane = minLane;
            // Debug.Log("Changed for " + i + " which is " + minLane);
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        // int n = m_envController.Sections.Length;
        // double mindist = 100000;
        // Vector3 minClosestPoint = new Vector3(0.0f,0.0f,0.0f);
        // for (int i = 0; i < n; i++)
        // {
        //     BoxCollider centerLine = m_envController.Sections[i].Trigger;
        //     Vector3 ClosestPoint = centerLine.ClosestPoint(m_Kart.transform.position);
        //     double dist = (ClosestPoint-m_Kart.transform.position).magnitude;
        //     if (dist<mindist)
        //     {
        //         mindist = dist;
        //         minClosestPoint = ClosestPoint;
        //     }
        // }
        // if (Vector3.Distance(minClosestPoint,lastClosestPoint)>0.1){
            
        //     // Debug.Log(minClosestPoint);
        //     writer.WriteLine(minClosestPoint.x+","+minClosestPoint.z+","+mindist);
        //     lastClosestPoint = minClosestPoint;
        // }
    }
}
