using KartGame.AI;
using KartGame.KartSystems;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Unity.MLAgents;
using UnityEngine;

public class CentreLineRecorder : MonoBehaviour
{
    public string path;
    StreamWriter writer;
    // public ArcadeKart m_Kart;
    public RacingEnvController m_envController;
    Vector3 lastClosestPoint;
    
    // Start is called before the first frame update
    void Start()
    {
        writer = new StreamWriter(path, true);
        var center_line_ = m_envController.Sections.SelectMany(s => s.finePoints).ToList();
        
        int n = center_line_.Count;
        for (int i = 0; i < n; i++)
        {
            Vector3 ClosestPoint = center_line_[i];
            writer.WriteLine(ClosestPoint.x+","+ClosestPoint.y+","+7.0f);
            Debug.Log(ClosestPoint.x+","+ClosestPoint.y+","+7.0f);
        }
        writer.Close();
        Debug.Log("File saved");
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
