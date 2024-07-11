using System;
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.VFX;
using Unity.MLAgents;
using KartGame.AI;

namespace KartGame.KartSystems
{
    public class ArcadeKart : MonoBehaviour
    {
        [System.Serializable]
        public class StatPowerup
        {
            public ArcadeKart.Stats modifiers;
            public string PowerUpID;
            public float ElapsedTime;
            public float MaxTime;
        }

        [System.Serializable]
        public struct VehicleDynamicParams
        {
            public float lf;
            public float lr;
            
            public float Df;
            public float Cf;
            public float Bf;
            
            public float Dr;
            public float Cr;
            public float Br;

            public float Cm1;
            public float Cm2;
            public float Cd;
            public float Cb;
            
            public float mass;
            public float Iz;
        }
        public float vx;
        public float vy;
        public int m_stepCount;
        public ArcadeKart.VehicleDynamicParams baseVehicleParams = new ArcadeKart.VehicleDynamicParams
        {
            lf = 0.3125f,
            lr = 0.3125f,
            Df = 10.0f,
            Cf = 1.5f,
            Bf = 4.0f,
            Dr = 10.0f,
            Cr = 1.5f,
            Br = 4.0f,
            Cm1 = 15.0f,
            Cm2 = 0.25f,
            Cd = 0.0f,
            Cb = 5.0f,
            mass = 1.0f,
            Iz = 0.04882813f
        };

        [System.Serializable]
        public struct Stats
        {
            [Header("Movement Settings")]
            [Min(0.001f), Tooltip("Top speed attainable when moving forward.")]
            public float TopSpeed;

            [Tooltip("How quickly the kart reaches top speed.")]
            public float Acceleration;

            [Min(0.001f), Tooltip("Top speed attainable when moving backward.")]
            public float ReverseSpeed;

            [Tooltip("How quickly the kart reaches top speed, when moving backward.")]
            public float ReverseAcceleration;

            [Tooltip("How quickly the kart starts accelerating from 0. A higher number means it accelerates faster sooner.")]
            [Range(0.2f, 1)]
            public float AccelerationCurve;

            [Tooltip("How quickly the kart slows down when the brake is applied.")]
            public float Braking;

            [Tooltip("How quickly the kart will reach a full stop when no inputs are made.")]
            public float CoastingDrag;

            [Range(0.0f, 1.0f)]
            [Tooltip("The amount of side-to-side friction.")]
            public float Grip;

            [Tooltip("How tightly the kart can turn left or right.")]
            public float Steer;

            [Tooltip("Max of how tightly the kart can turn left or right.")]
            public float MaxSteer;

            [Tooltip("Min of how tightly the kart can turn left or right.")]
            public float MinSteer;

            [Tooltip("Coeffecient of tire wear for this car")]
            public float TireWearFactor;

            [Tooltip("Minimum Lateral Gs the car will always be able to sustain")]
            public float MinGs;

            [Tooltip("Maximum Lateral Gs of the tire performance")]
            public float MaxGs;

            [Tooltip("Additional gravity for when the kart is in the air.")]
            public float AddedGravity;

            // allow for stat adding for powerups.
            public static Stats operator +(Stats a, Stats b)
            {
                return new Stats
                {
                    Acceleration = a.Acceleration + b.Acceleration,
                    AccelerationCurve = a.AccelerationCurve + b.AccelerationCurve,
                    Braking = a.Braking + b.Braking,
                    CoastingDrag = a.CoastingDrag + b.CoastingDrag,
                    AddedGravity = a.AddedGravity + b.AddedGravity,
                    Grip = a.Grip + b.Grip,
                    ReverseAcceleration = a.ReverseAcceleration + b.ReverseAcceleration,
                    ReverseSpeed = a.ReverseSpeed + b.ReverseSpeed,
                    TopSpeed = a.TopSpeed + b.TopSpeed,
                    MaxSteer = a.MaxSteer + b.MaxSteer,
                    MinSteer = a.MinSteer + b.MinSteer,
                    Steer = a.Steer + b.Steer,
                    TireWearFactor = a.TireWearFactor + b.TireWearFactor,
                    MinGs = a.MinGs + b.MinGs,
                    MaxGs = a.MaxGs + b.MaxGs
                };
            }
        }

        public Rigidbody Rigidbody { get; private set; }
        public InputData Input     { get; private set; }
        public float AirPercent    { get; private set; }
        public float GroundPercent { get; private set; }
        public bool useDynamicModel = true;
        public ArcadeKart.Stats baseStats = new ArcadeKart.Stats
        {
            TopSpeed            = 10f,
            Acceleration        = 5f,
            AccelerationCurve   = 4f,
            Braking             = 10f,
            ReverseAcceleration = 5f,
            ReverseSpeed        = 5f,
            Steer               = 5f,
            MaxSteer            = 5f,
            MinSteer            = 1f,
            CoastingDrag        = 4f,
            Grip                = .95f,
            AddedGravity        = 1f,
            TireWearFactor      = 0.0001f,
            MinGs               = 0.5f,
            MaxGs               = 1.5f
        };

        [Header("Vehicle Visual")] 
        public List<GameObject> m_VisualWheels;

        public HierarchicalKartAgent hc; 
        public float t = 0.0f;
        [Header("Vehicle Physics")]
        [Tooltip("The transform that determines the position of the kart's mass.")]
        public Transform CenterOfMass;

        public bool takeManualCommands = false;

        [Range(0.0f, 20.0f), Tooltip("Coefficient used to reorient the kart in the air. The higher the number, the faster the kart will readjust itself along the horizontal plane.")]
        public float AirborneReorientationCoefficient = 3.0f;

        [Header("Drifting")]
        [Range(0.01f, 1.0f), Tooltip("The grip value when drifting.")]
        public float DriftGrip = 0.4f;
        [Range(0.0f, 10.0f), Tooltip("Additional steer when the kart is drifting.")]
        public float DriftAdditionalSteer = 5.0f;
        [Range(1.0f, 30.0f), Tooltip("The higher the angle, the easier it is to regain full grip.")]
        public float MinAngleToFinishDrift = 10.0f;
        [Range(0.01f, 0.99f), Tooltip("Mininum speed percentage to switch back to full grip.")]
        public float MinSpeedPercentToFinishDrift = 0.5f;
        [Range(1.0f, 20.0f), Tooltip("The higher the value, the easier it is to control the drift steering.")]
        public float DriftControl = 10.0f;
        [Range(0.0f, 20.0f), Tooltip("The lower the value, the longer the drift will last without trying to control it by steering.")]
        public float DriftDampening = 10.0f;

        [Header("VFX")]
        [Tooltip("VFX that will be placed on the wheels when drifting.")]
        public ParticleSystem DriftSparkVFX;
        [Range(0.0f, 0.2f), Tooltip("Offset to displace the VFX to the side.")]
        public float DriftSparkHorizontalOffset = 0.1f;
        [Range(0.0f, 90.0f), Tooltip("Angle to rotate the VFX.")]
        public float DriftSparkRotation = 17.0f;
        [Tooltip("VFX that will be placed on the wheels when drifting.")]
        public GameObject DriftTrailPrefab;
        [Range(-0.1f, 0.1f), Tooltip("Vertical to move the trails up or down and ensure they are above the ground.")]
        public float DriftTrailVerticalOffset;
        [Tooltip("VFX that will spawn upon landing, after a jump.")]
        public GameObject JumpVFX;
        [Tooltip("VFX that is spawn on the nozzles of the kart.")]
        public GameObject NozzleVFX;

        public Transform LeftHeadLight;
        
        public Transform RightHeadLight;

        [Tooltip("List of the kart's nozzles.")]
        public List<Transform> Nozzles;

        [Header("Suspensions")]
        [Tooltip("The maximum extension possible between the kart's body and the wheels.")]
        [Range(0.0f, 1.0f)]
        public float SuspensionHeight = 0.2f;
        [Range(10.0f, 100000.0f), Tooltip("The higher the value, the stiffer the suspension will be.")]
        public float SuspensionSpring = 20000.0f;
        [Range(0.0f, 5000.0f), Tooltip("The higher the value, the faster the kart will stabilize itself.")]
        public float SuspensionDamp = 500.0f;
        [Tooltip("Vertical offset to adjust the position of the wheels relative to the kart's body.")]
        [Range(-1.0f, 1.0f)]
        public float WheelsPositionVerticalOffset = 0.0f;

        [Header("Physical Wheels")]
        [Tooltip("The physical representations of the Kart's wheels.")]
        public WheelCollider FrontLeftWheel;
        public WheelCollider FrontRightWheel;
        public WheelCollider RearLeftWheel;
        public WheelCollider RearRightWheel;

        [Tooltip("Which layers the wheels will detect.")]
        public LayerMask GroundLayers = Physics.DefaultRaycastLayers;

        // the input sources that can control the kart
        IInput[] m_Inputs;

        const float k_NullInput = 0.01f;
        const float k_NullSpeed = 0.01f;
        Vector3 m_VerticalReference = Vector3.up;

        float m_CurrentGrip = 1.0f;
        [HideInInspector] public float m_AccumulatedAngularV = 0.0f;
        [HideInInspector] public float TireWearRate = 10000.0f;
        float m_PreviousGroundPercent = 1.0f;

        // can the kart move?
        bool m_CanMove = true;
        List<StatPowerup> m_ActivePowerupList = new List<StatPowerup>();
        [HideInInspector] public ArcadeKart.Stats m_FinalStats;

        Quaternion m_LastValidRotation;
        Vector3 m_LastValidPosition;
        Vector3 m_LastCollisionNormal;
        bool m_HasCollision;
        public bool isDrifting = false;

        bool m_InAir = false;
        readonly List<(GameObject trailRoot, WheelCollider wheel, TrailRenderer trail)> m_DriftTrailInstances = new List<(GameObject, WheelCollider, TrailRenderer)>();
        readonly List<(WheelCollider wheel, float horizontalOffset, float rotation, ParticleSystem sparks)> m_DriftSparkInstances = new List<(WheelCollider, float, float, ParticleSystem)>();

        [HideInInspector] public Vector3 acc = Vector3.zero;


        public void AddPowerup(StatPowerup statPowerup) => m_ActivePowerupList.Add(statPowerup);
        public void SetCanMove(bool move) => m_CanMove = move;
        public float GetMaxSpeed() => Mathf.Max(m_FinalStats.TopSpeed, m_FinalStats.ReverseSpeed);

        private void ActivateDriftVFX(bool active)
        {
            foreach (var vfx in m_DriftSparkInstances)
            {
                if (active)// && vfx.wheel.GetGroundHit(out WheelHit hit))
                {
                    if (!vfx.sparks.isPlaying)
                        vfx.sparks.Play();
                }
                else
                {
                    if (vfx.sparks.isPlaying)
                        vfx.sparks.Stop(true, ParticleSystemStopBehavior.StopEmitting);
                }
                    
            }

            foreach (var trail in m_DriftTrailInstances)
                trail.Item3.emitting = active;// && trail.wheel.GetGroundHit(out WheelHit hit);
        }

        private void UpdateDriftVFXOrientation()
        {
            foreach (var vfx in m_DriftSparkInstances)
            {
                vfx.sparks.transform.position = vfx.wheel.transform.position - (vfx.wheel.radius * Vector3.up) + (DriftTrailVerticalOffset * Vector3.up) + (transform.right * vfx.horizontalOffset);
                vfx.sparks.transform.rotation = transform.rotation * Quaternion.Euler(0.0f, 0.0f, vfx.rotation);
            }

            foreach (var trail in m_DriftTrailInstances)
            {
                trail.trailRoot.transform.position = trail.wheel.transform.position - (trail.wheel.radius * Vector3.up) + (DriftTrailVerticalOffset * Vector3.up);
                trail.trailRoot.transform.rotation = transform.rotation;
            }
        }

        void UpdateSuspensionParams(WheelCollider wheel)
        {
            wheel.suspensionDistance = SuspensionHeight;
            wheel.center = new Vector3(0.0f, WheelsPositionVerticalOffset, 0.0f);
            JointSpring spring = wheel.suspensionSpring;
            spring.spring = SuspensionSpring;
            spring.damper = SuspensionDamp;
            wheel.suspensionSpring = spring;
            // Debug.Log("Suspension updated");
        }

        void Awake()
        {
            Rigidbody = GetComponent<Rigidbody>();
            m_Inputs = GetComponents<IInput>();

            UpdateSuspensionParams(FrontLeftWheel);
            UpdateSuspensionParams(FrontRightWheel);
            UpdateSuspensionParams(RearLeftWheel);
            UpdateSuspensionParams(RearRightWheel);

            if (DriftSparkVFX != null)
            {
                Debug.Log("Spark added");
                AddSparkToWheel(RearLeftWheel, -DriftSparkHorizontalOffset, -DriftSparkRotation);
                AddSparkToWheel(RearRightWheel, DriftSparkHorizontalOffset, DriftSparkRotation);
            }

            if (DriftTrailPrefab != null)
            {
                AddTrailToWheel(RearLeftWheel);
                AddTrailToWheel(RearRightWheel);
            }

            m_CurrentGrip = baseStats.Grip;
            if (NozzleVFX != null)
            {
                foreach (var nozzle in Nozzles)
                {
                    Instantiate(NozzleVFX, nozzle, false);
                }
            }
            UpdateStats();
        }

        void AddTrailToWheel(WheelCollider wheel)
        {
            GameObject trailRoot = Instantiate(DriftTrailPrefab, wheel.transform, false);
            
            TrailRenderer trail = trailRoot.GetComponentInChildren<TrailRenderer>();
            trail.emitting = false;
            m_DriftTrailInstances.Add((trailRoot, wheel, trail));
        }

        void AddSparkToWheel(WheelCollider wheel, float horizontalOffset, float rotation)
        {
            GameObject vfx = Instantiate(DriftSparkVFX.gameObject, wheel.transform, false);
            ParticleSystem spark = vfx.GetComponent<ParticleSystem>();
            spark.Stop();
            m_DriftSparkInstances.Add((wheel, horizontalOffset, -rotation, spark));
        }

        void FixedUpdate()
        {
            UpdateSuspensionParams(FrontLeftWheel);
            UpdateSuspensionParams(FrontRightWheel);
            UpdateSuspensionParams(RearLeftWheel);
            UpdateSuspensionParams(RearRightWheel);
            m_stepCount = Academy.Instance.TotalStepCount*8;

            GatherInputs();
            UpdateStats();
            // apply our physics properties
            Rigidbody.centerOfMass = transform.InverseTransformPoint(CenterOfMass.position);

            int groundedCount = 0;
            if (FrontLeftWheel.isGrounded && FrontLeftWheel.GetGroundHit(out WheelHit hit))
                groundedCount++;
            if (FrontRightWheel.isGrounded && FrontRightWheel.GetGroundHit(out hit))
                groundedCount++;
            if (RearLeftWheel.isGrounded && RearLeftWheel.GetGroundHit(out hit))
                groundedCount++;
            if (RearRightWheel.isGrounded && RearRightWheel.GetGroundHit(out hit))
                groundedCount++;

            // calculate how grounded and airborne we are
            GroundPercent = (float) groundedCount / 4.0f;
            AirPercent = 1 - GroundPercent;

            // apply vehicle physics
            if (m_CanMove)
            {
                // print("Moving vehicle ");
                if (useDynamicModel) MoveVehicleDynamic(Input.Accelerate, Input.Brake, Input.TurnInput);
                else MoveVehicle(Input.Accelerate, Input.Brake, Input.TurnInput);
            }
            GroundAirbourne();

            m_PreviousGroundPercent = GroundPercent;

        }

        void GatherInputs()
        {
            // reset input
            Input = new InputData();

            int n = m_Inputs.Length;
            // gather nonzero input from our sources
            if(takeManualCommands) n = 1;
            for (int i = 0; i < n; i++)
            {
                Input = m_Inputs[i].GenerateInput();
                
                // if(takeManualCommands) Debug.Log("Collected input : " + Input.Accelerate + " " + Input.Brake + " " + Input.TurnInput);
                // print("Input " + i + " accelerate is " + Input.Accelerate);
                // print("Input " + i + " brake is " + Input.Brake);
            }
        }

        public void UpdateStats()
        {
            m_FinalStats = baseStats;
            // clamp values in finalstats
            m_FinalStats.Grip = Mathf.Clamp(m_FinalStats.Grip, 0, 1);
            // m_FinalStats.Steer = Mathf.Clamp((baseStats.MaxSteer * Mathf.Exp(-m_AccumulatedAngularV/TireWearRate)), baseStats.MinSteer, baseStats.MaxSteer);
            m_FinalStats.Steer = Mathf.Clamp((baseStats.MaxSteer), baseStats.MinSteer, baseStats.MaxSteer);
            // print(m_FinalStats.Steer);
        }

        public float TireWearProportion()
        {
            return (baseStats.MaxSteer - m_FinalStats.Steer) / (baseStats.MaxSteer - baseStats.MinSteer);
        }

        void GroundAirbourne()
        {
            // while in the air, fall faster
            if (AirPercent >= 1)
            {
                Rigidbody.velocity += Physics.gravity * Time.fixedDeltaTime * m_FinalStats.AddedGravity;
            }
        }

        public void Reset()
        {
            Vector3 euler = transform.rotation.eulerAngles;
            euler.x = euler.z = 0f;
            transform.rotation = Quaternion.Euler(euler);
        }

        public float LocalSpeed()
        {
            if (m_CanMove)
            {
                float dot = Vector3.Dot(transform.forward, Rigidbody.velocity);
                if (Mathf.Abs(dot) > 0.1f)
                {
                    float speed = Rigidbody.velocity.magnitude;
                    return dot < 0 ? -(speed / m_FinalStats.ReverseSpeed) : (speed / m_FinalStats.TopSpeed);
                }
                return 0f;
            }
            else
            {
                // use this value to play kart sound when it is waiting the race start countdown.
                return  0.0f;
            }
        }
        void OnEnable()
        {
            UpdateStats();
        }
        void OnCollisionEnter(Collision collision) => m_HasCollision = true;
        void OnCollisionExit(Collision collision) => m_HasCollision = false;

        void OnCollisionStay(Collision collision)
        {
            m_HasCollision = true;
            m_LastCollisionNormal = Vector3.zero;
            float dot = -1.0f;

            foreach (var contact in collision.contacts)
            {
                if (Vector3.Dot(contact.normal, Vector3.up) > dot)
                    m_LastCollisionNormal = contact.normal;
            }
        }
        void MoveVehicleDynamic_s(bool accelerate, bool brake, float turnInput)
        {
            // turnInput assumes it is equiavalent steering angle * vx  to model dynamic bicycle model physics with slippage consideration. 
            // the LQNG-based agents compute the turnInput from the yaw rate output by the solver (yaw_dot) and apply the inverse of the equation
            // more details on the kinematic bicycle model are here: Vehicle Dynamics and Control by Rajesh Rajamani
            
            // print("actual moving Accelerate " + accelerate + " Brake: " + brake + " turning input: " + turnInput);
            // Debug.Log("Haha");

            // manual acceleration curve coefficient scalar
            // float accelerationCurveCoeff = 5;
            Vector3 localVel = transform.InverseTransformVector(Rigidbody.velocity);

            float vx = localVel.z;
            float vy = localVel.x;
            var angularVel = Rigidbody.angularVelocity;
            float w = angularVel.y;
            
            // float steerInput = turnInput/Mathf.Max(vx,1.0f);
            float steerInput = turnInput/12.0f;
            float accelInput = (accelerate ? 1.0f : 0.0f) - (brake ? 1.0f : 0.0f);
            
            float alpha_f = steerInput - Mathf.Atan2(w*baseVehicleParams.lf+vy,Mathf.Max(vx,3.0f));
            float alpha_r = Mathf.Atan2(w*baseVehicleParams.lr-vy,Mathf.Max(vx,3.0f));
            // alpha_f = Mathf.Max(Mathf.Min(alpha_f,0.6f),-0.6f);
            // alpha_r = Mathf.Max(Mathf.Min(alpha_r,0.6f),-0.6f);
            // t = Mathf.Max(0.0f,Mathf.Min(1.0f,(m_stepCount-500000)/1500000));
            t = Mathf.Max(0.0f,Mathf.Min(1.0f,(m_stepCount-500000)/500000));
            float Df = baseVehicleParams.Df*Mathf.Pow(3,1-t);
            float Cf = baseVehicleParams.Cf/(Mathf.Pow(baseVehicleParams.Cf,2-2*t));
            float Bf = Mathf.Pow(2,1-t)*baseVehicleParams.Df*baseVehicleParams.Cf*baseVehicleParams.Bf/(Df*Cf);
            float Dr = baseVehicleParams.Dr*Mathf.Pow(3,1-t);
            float Cr = baseVehicleParams.Cr/(Mathf.Pow(baseVehicleParams.Cr,2-2*t));
            float Br = Mathf.Pow(2,1-t)*baseVehicleParams.Dr*baseVehicleParams.Cr*baseVehicleParams.Br/(Dr*Cr);
            // isDrifting = false;
            // Bfn = 2**(1-t)*Df*Cf*Bf/(Dfn*Cfn)
            // float Ffy = baseVehicleParams.Df*Mathf.Sin(baseVehicleParams.Cf*Mathf.Atan(baseVehicleParams.Bf*alpha_f));
            // float Fry = baseVehicleParams.Dr*Mathf.Sin(baseVehicleParams.Cr*Mathf.Atan(baseVehicleParams.Br*alpha_r));
            float Ffy = Df*Mathf.Sin(Cf*Mathf.Atan(Bf*alpha_f));
            float Fry = Dr*Mathf.Sin(Cr*Mathf.Atan(Br*alpha_r));
            
            
            float Frx = (accelInput > 0) ? (baseVehicleParams.Cm1-baseVehicleParams.Cm2*vx)*accelInput - baseVehicleParams.Cd*vx*vx : baseVehicleParams.Cb*accelInput - baseVehicleParams.Cd*vx*vx;
            float vx_dot = (Frx - ((vx>3.0f)?Ffy*Mathf.Sin(steerInput):0.0f) + baseVehicleParams.mass*vy*w)/baseVehicleParams.mass;
            float vy_dot = (Fry + Ffy*Mathf.Cos(steerInput) - baseVehicleParams.mass*vx*w)/baseVehicleParams.mass;
            float w_dot = (Ffy*baseVehicleParams.lf*Mathf.Cos(steerInput) - Fry*baseVehicleParams.lr)/baseVehicleParams.Iz;
            
            Vector3 forwardDir = transform.forward;
            Vector3 rightDir = transform.right;

            // Debug.Log(accelerate+ " " +brake+" "+turnInput+" "+alpha_f+" "+alpha_r);
            Vector3 v_dot = vx_dot*forwardDir + ((vx>3.0f)?vy_dot:-2.0f*vy)*rightDir;
            w = ((vx>3.0f)?w:((vx<-10.0f)?0.0f:steerInput*vx/(baseVehicleParams.lf+baseVehicleParams.lr)));
            Vector3 newVelocity = Quaternion.AngleAxis(w*Time.fixedDeltaTime*180.0f/Mathf.PI, transform.up)*(Rigidbody.velocity + v_dot * Time.fixedDeltaTime);
            // forwardDir = Quaternion.AngleAxis(w*Time.fixedDeltaTime*180.0f/Mathf.PI, transform.up)*forwardDir;
            // rightDir = Quaternion.AngleAxis(w*Time.fixedDeltaTime*180.0f/Mathf.PI, transform.up)*rightDir;
            newVelocity.y = Rigidbody.velocity.y;
            // move the Y angular velocity towards our target
            angularVel.y = w + ((vx>3.0f)?(Time.fixedDeltaTime*w_dot):0.0f);
            m_AccumulatedAngularV += Mathf.Abs(angularVel.y);
            // apply the angular velocity
            // Debug.Log(alpha_f + " " + alpha_r);
            if (GroundPercent>0){
                if (Mathf.Abs(alpha_f+alpha_r) > 0.2f) isDrifting = true; 
                else isDrifting = false;
                Rigidbody.velocity = newVelocity;
                Rigidbody.angularVelocity = angularVel;
            }
            else
            {
                m_InAir = true;
            }
            ActivateDriftVFX(isDrifting);
            if (LeftHeadLight!=null&&RightHeadLight!=null){
                if (brake) {
                    LeftHeadLight.gameObject.SetActive(true);
                    RightHeadLight.gameObject.SetActive(true);
                }
                else {
                    LeftHeadLight.gameObject.SetActive(false);
                    RightHeadLight.gameObject.SetActive(false);
                }
            }
        }


        public void ApplyCBFWall(float turnInput, float accInput, out float turnOutput, out float accOutput){
            Vector3 localVel = transform.InverseTransformVector(Rigidbody.velocity);

            vx = localVel.z;
            vy = localVel.x;
            var angularVel = Rigidbody.angularVelocity;
            float w = angularVel.y;
            // Debug.Log(turnInput);
            // float steerInput = turnInput/Mathf.Max(vx,1.0f);
            float cost = 0.0f;
            float t_ = 1.0f;//Mathf.Max(0.0f,Mathf.Min(1.0f,(m_stepCount-500000)/1500000));
            float Df = baseVehicleParams.Df*Mathf.Pow(3,1-t_);
            float Cf = baseVehicleParams.Cf/(Mathf.Pow(baseVehicleParams.Cf,2-2*t_));
            float Bf = Mathf.Pow(2,1-t_)*baseVehicleParams.Df*baseVehicleParams.Cf*baseVehicleParams.Bf/(Df*Cf);
            float Dr = baseVehicleParams.Dr*Mathf.Pow(3,1-t_);
            float Cr = baseVehicleParams.Cr/(Mathf.Pow(baseVehicleParams.Cr,2-2*t_));
            float Br = Mathf.Pow(2,1-t_)*baseVehicleParams.Dr*baseVehicleParams.Cr*baseVehicleParams.Br/(Dr*Cr);
            float mincost = -1.0f;
            float optimal_a = -1.0f;
            float optimal_t = 0.0f;
            float lambda_1 = 2.0f, lambda_2 = 2.0f, TrackWidth=3.5f;
            for (float a = -1.0f; a<1.01f; a+=1.0f){
                for (float t = -1.0f; t<1.001f; t+=2.0f/50.0f){
                    cost = 10.0f*Mathf.Pow(t-turnInput,2) + Mathf.Pow(a-accInput,2);
                    float steerInput = t/Mathf.Max(vx,1.0f);
                    float alpha_f = steerInput - Mathf.Atan2(w*baseVehicleParams.lf+vy,Mathf.Max(vx,3.0f));
                    float alpha_r = Mathf.Atan2(w*baseVehicleParams.lr-vy,Mathf.Max(vx,3.0f));
                    float Ffy = Df*Mathf.Sin(Cf*Mathf.Atan(Bf*alpha_f));
                    float Fry = Dr*Mathf.Sin(Cr*Mathf.Atan(Br*alpha_r));        
                    float Frx = (a > 0) ? (baseVehicleParams.Cm1-baseVehicleParams.Cm2*vx)*a - baseVehicleParams.Cd*vx*vx : baseVehicleParams.Cb*a - baseVehicleParams.Cd*vx*vx;
                    float vx_dot = (Frx - ((vx>3.0f)?Ffy*Mathf.Sin(steerInput):0.0f) + baseVehicleParams.mass*vy*w)/baseVehicleParams.mass;
                    float vy_dot = (Fry + Ffy*Mathf.Cos(steerInput) - baseVehicleParams.mass*vx*w)/baseVehicleParams.mass;
                    // float w_dot = (Ffy*baseVehicleParams.lf*Mathf.Cos(steerInput) - Fry*baseVehicleParams.lr)/baseVehicleParams.Iz;
                    
                    // Left boundary CBF
                    float h = TrackWidth - hc.dist_centerline;
                    float h_dot = -(vy*Mathf.Cos(hc.rel_angle_center)+vx*Mathf.Sin(hc.rel_angle_center));
                    float h_dot_dot = -(vy_dot*Mathf.Cos(hc.rel_angle_center)+vx_dot*Mathf.Sin(hc.rel_angle_center))
                                    +(vy_dot*Mathf.Sin(hc.rel_angle_center)-vx_dot*Mathf.Cos(hc.rel_angle_center))*(w-(vx*Mathf.Cos(hc.rel_angle_center)-vy*Mathf.Sin(hc.rel_angle_center))*hc.curv_center);
                    float viol = h + (lambda_1+lambda_2)*h_dot + lambda_1*lambda_2*h_dot_dot;
                    if (viol<0.0f) cost += 10000.0f*viol*viol;
                
                    // Right boundary CBF
                    h = TrackWidth + hc.dist_centerline;
                    h_dot = (vy*Mathf.Cos(hc.rel_angle_center)+vx*Mathf.Sin(hc.rel_angle_center));
                    h_dot_dot = (vy_dot*Mathf.Cos(hc.rel_angle_center)+vx_dot*Mathf.Sin(hc.rel_angle_center))
                                    -(vy_dot*Mathf.Sin(hc.rel_angle_center)-vx_dot*Mathf.Cos(hc.rel_angle_center))*(w-(vx*Mathf.Cos(hc.rel_angle_center)-vy*Mathf.Sin(hc.rel_angle_center))*hc.curv_center);
                    viol = h + (lambda_1+lambda_2)*h_dot + lambda_1*lambda_2*h_dot_dot;
                    if (viol<0.0f) cost += 10000.0f*viol*viol;
                    if (cost<mincost || mincost < 0.0f){
                        mincost = cost;
                        optimal_a = a;
                        optimal_t = t;
                    }
                }

            }
            turnOutput = optimal_t;
            accOutput = optimal_a; 
        }
        
        void MoveVehicleDynamic(bool accelerate, bool brake, float turnInput)
        {
            // turnInput assumes it is equiavalent steering angle * vx  to model dynamic bicycle model physics with slippage consideration. 
            // the LQNG-based agents compute the turnInput from the yaw rate output by the solver (yaw_dot) and apply the inverse of the equation
            // more details on the kinematic bicycle model are here: Vehicle Dynamics and Control by Rajesh Rajamani
            
            // print("actual moving Accelerate " + accelerate + " Brake: " + brake + " turning input: " + turnInput);
            // Debug.Log("Haha");

            // manual acceleration curve coefficient scalar
            // float accelerationCurveCoeff = 5;
            Vector3 localVel = transform.InverseTransformVector(Rigidbody.velocity);

            float vx = localVel.z;
            float vy = localVel.x;
            var angularVel = Rigidbody.angularVelocity;
            float w = angularVel.y;
            float turnOutput = 0.0f, accOutput = -1.0f;
            // float steerInput = turnInput/Mathf.Max(vx,1.0f);
            float accelInput = (accelerate ? 1.0f : 0.0f) - (brake ? 1.0f : 0.0f);
            // ApplyCBFWall(turnInput,accelInput,out turnOutput,out accOutput);
            // accelInput = accOutput;
            // float steerInput = turnOutput/Mathf.Max(vx,1.0f);
            float steerInput = turnInput/Mathf.Max(vx,1.0f);
            float alpha_f = steerInput - Mathf.Atan2(w*baseVehicleParams.lf+vy,Mathf.Max(vx,3.0f));
            float alpha_r = Mathf.Atan2(w*baseVehicleParams.lr-vy,Mathf.Max(vx,3.0f));
            // alpha_f = Mathf.Max(Mathf.Min(alpha_f,0.6f),-0.6f);
            // alpha_r = Mathf.Max(Mathf.Min(alpha_r,0.6f),-0.6f);
            // t = Mathf.Max(0.0f,Mathf.Min(1.0f,(m_stepCount-500000)/1500000));
            t = Mathf.Max(0.0f,Mathf.Min(1.0f,(m_stepCount-500000)/1500000));
            float Df = baseVehicleParams.Df*Mathf.Pow(3,1-t);
            // Debug.Log(Df+" "+t);
            float Cf = baseVehicleParams.Cf/(Mathf.Pow(baseVehicleParams.Cf,2-2*t));
            float Bf = Mathf.Pow(2,1-t)*baseVehicleParams.Df*baseVehicleParams.Cf*baseVehicleParams.Bf/(Df*Cf);
            float Dr = baseVehicleParams.Dr*Mathf.Pow(3,1-t);
            float Cr = baseVehicleParams.Cr/(Mathf.Pow(baseVehicleParams.Cr,2-2*t));
            float Br = Mathf.Pow(2,1-t)*baseVehicleParams.Dr*baseVehicleParams.Cr*baseVehicleParams.Br/(Dr*Cr);
            // isDrifting = false;
            // Bfn = 2**(1-t)*Df*Cf*Bf/(Dfn*Cfn)
            // float Ffy = baseVehicleParams.Df*Mathf.Sin(baseVehicleParams.Cf*Mathf.Atan(baseVehicleParams.Bf*alpha_f));
            // float Fry = baseVehicleParams.Dr*Mathf.Sin(baseVehicleParams.Cr*Mathf.Atan(baseVehicleParams.Br*alpha_r));
            float Ffy = Df*Mathf.Sin(Cf*Mathf.Atan(Bf*alpha_f));
            float Fry = Dr*Mathf.Sin(Cr*Mathf.Atan(Br*alpha_r));
            
            
            float Frx = (accelInput > 0) ? (baseVehicleParams.Cm1-baseVehicleParams.Cm2*vx)*accelInput - baseVehicleParams.Cd*vx*vx : baseVehicleParams.Cb*accelInput - baseVehicleParams.Cd*vx*vx;
            float vx_dot = (Frx - ((vx>3.0f)?Ffy*Mathf.Sin(steerInput):0.0f) + baseVehicleParams.mass*vy*w)/baseVehicleParams.mass;
            float vy_dot = (Fry + Ffy*Mathf.Cos(steerInput) - baseVehicleParams.mass*vx*w)/baseVehicleParams.mass;
            float w_dot = (Ffy*baseVehicleParams.lf*Mathf.Cos(steerInput) - Fry*baseVehicleParams.lr)/baseVehicleParams.Iz;
            
            // Debug.Log(steerInput + " " + accelInput + " " + vx_dot + " " + vy_dot + " " + Ffy + " "+Bf+ " "+Cf+ " "+Df+ " " + vx + " " + vy + " " + w);
            Vector3 forwardDir = transform.forward;
            Vector3 rightDir = transform.right;

            // Debug.Log(accelerate+ " " +brake+" "+turnInput+" "+alpha_f+" "+alpha_r);
            Vector3 v_dot = vx_dot*forwardDir + ((vx>3.0f)?vy_dot:-2.0f*vy)*rightDir;
            w = ((vx>3.0f)?w:((vx<-10.0f)?0.0f:steerInput*vx/(baseVehicleParams.lf+baseVehicleParams.lr)));
            Vector3 newVelocity = Quaternion.AngleAxis(w*Time.fixedDeltaTime*180.0f/Mathf.PI, transform.up)*(Rigidbody.velocity + v_dot * Time.fixedDeltaTime);
            // forwardDir = Quaternion.AngleAxis(w*Time.fixedDeltaTime*180.0f/Mathf.PI, transform.up)*forwardDir;
            // rightDir = Quaternion.AngleAxis(w*Time.fixedDeltaTime*180.0f/Mathf.PI, transform.up)*rightDir;
            newVelocity.y = Rigidbody.velocity.y;
            // move the Y angular velocity towards our target
            angularVel.y = w + ((vx>3.0f)?(Time.fixedDeltaTime*w_dot):0.0f);
            m_AccumulatedAngularV += Mathf.Abs(angularVel.y);
            // apply the angular velocity
            // Debug.Log(alpha_f + " " + alpha_r);
            if (GroundPercent>0){
                if (Mathf.Abs(alpha_f+alpha_r) > 0.2f) isDrifting = true; 
                else isDrifting = false;
                Rigidbody.velocity = newVelocity;
                Rigidbody.angularVelocity = angularVel;
            }
            else
            {
                m_InAir = true;
            }
            ActivateDriftVFX(isDrifting);
            if (LeftHeadLight!=null&&RightHeadLight!=null){
                if (brake) {
                    LeftHeadLight.gameObject.SetActive(true);
                    RightHeadLight.gameObject.SetActive(true);
                }
                else {
                    LeftHeadLight.gameObject.SetActive(false);
                    RightHeadLight.gameObject.SetActive(false);
                }
            }
        }

        void MoveVehicle(bool accelerate, bool brake, float turnInput)
        {
            // turnInput assumes it is equiavalent v*tan(steering angle) to model kinematic bicycle model physics with no slippage. yaw_dot = v*tan(steering_angle)/L
            // v is current velocity of vehicle and steering angle is desired steering angle and L is length of kart.
            // the RL-based agents learn how to incorporate this when producing the turn input commands
            // the LQNG-based agents compute the turnInput from the yaw rate output by the solver (yaw_dot) and apply the inverse of the equation
            // more details on the kinematic bicycle model are here: Vehicle Dynamics and Control by Rajesh Rajamani
            
            // print("actual moving Accelerate " + accelerate + " Brake: " + brake + " turning input: " + turnInput);
            // Debug.Log("Haha");
            float accelInput = (accelerate ? 1.0f : 0.0f) - (brake ? 1.0f : 0.0f);

            // manual acceleration curve coefficient scalar
            float accelerationCurveCoeff = 5;
            Vector3 localVel = transform.InverseTransformVector(Rigidbody.velocity);

            bool accelDirectionIsFwd = accelInput >= 0;
            bool localVelDirectionIsFwd = localVel.z >= 0;

            // use the max speed for the direction we are going--forward or reverse.
            float maxSpeed = localVelDirectionIsFwd ? m_FinalStats.TopSpeed : m_FinalStats.ReverseSpeed;
            var maxAllowedSpeed = Mathf.Sqrt(getMaxLateralGs() * 9.81f * (Mathf.Abs(getTurningRadius())));
            // print(maxAllowedSpeed);
            if (!(float.IsInfinity(maxAllowedSpeed) || float.IsNaN(maxAllowedSpeed)))
            {
                maxSpeed = Mathf.Clamp(maxSpeed, 0.001f, Mathf.Max(maxAllowedSpeed, 0.001f));
            }
            float accelPower = accelDirectionIsFwd ? m_FinalStats.Acceleration : m_FinalStats.ReverseAcceleration;

            float currentSpeed = Rigidbody.velocity.magnitude;
            float accelRampT = currentSpeed / maxSpeed;
            float multipliedAccelerationCurve = m_FinalStats.AccelerationCurve * accelerationCurveCoeff;
            float accelRamp = Mathf.Lerp(multipliedAccelerationCurve, 1, accelRampT * accelRampT);

            bool isBraking = (localVelDirectionIsFwd && brake) || (!localVelDirectionIsFwd && accelerate);

            // if we are braking (moving reverse to where we are going)
            // use the braking accleration instead
            float finalAccelPower = isBraking ? m_FinalStats.Braking : accelPower;

            float finalAcceleration = finalAccelPower * accelRamp;

            // apply inputs to forward/backward
            float turningPower = turnInput * m_FinalStats.Steer * (Mathf.Abs(currentSpeed) > 0.5f ? 1f: 0f);

            Quaternion turnAngle = Quaternion.AngleAxis(turningPower, transform.up);
            Vector3 fwd = turnAngle * transform.forward;
            acc = fwd * accelInput * finalAcceleration * ((m_HasCollision || GroundPercent > 0.0f) ? 1.0f : 0.0f);

            // forward movement
            bool wasOverMaxSpeed = currentSpeed >= maxSpeed;

            // if over max speed, cannot accelerate faster.
            if (wasOverMaxSpeed && !isBraking) 
                acc *= 0.0f;

            Vector3 newVelocity = (Rigidbody.velocity + acc * Time.fixedDeltaTime);
            newVelocity.y = Rigidbody.velocity.y;

            //  clamp max speed if we are on ground
            if (GroundPercent > 0.0f && wasOverMaxSpeed)
            {
                newVelocity = Vector3.ClampMagnitude(newVelocity, maxSpeed);
            }

            // coasting is when we aren't touching accelerate
            if (Mathf.Abs(accelInput) < k_NullInput && GroundPercent > 0.0f)
            {
                newVelocity = Vector3.MoveTowards(newVelocity, new Vector3(0, Rigidbody.velocity.y, 0), Time.fixedDeltaTime * m_FinalStats.CoastingDrag);
            }

            Rigidbody.velocity = newVelocity;

            // Drift
            if (GroundPercent > 0.0f)
            {
                if (m_InAir)
                {
                    m_InAir = false;
                    // Instantiate(JumpVFX, transform.position, Quaternion.identity);
                }

                // manual angular velocity coefficient
                float angularVelocitySteering = 0.4f;
                float angularVelocitySmoothSpeed = 20f;

                // turning is reversed if we're going in reverse and pressing reverse
                if (!localVelDirectionIsFwd && !accelDirectionIsFwd) 
                    angularVelocitySteering *= -1.0f;

                var angularVel = Rigidbody.angularVelocity;

                // move the Y angular velocity towards our target
                angularVel.y = Mathf.MoveTowards(angularVel.y, turningPower * angularVelocitySteering, Time.fixedDeltaTime * angularVelocitySmoothSpeed);
                m_AccumulatedAngularV += Mathf.Abs(angularVel.y);
                // apply the angular velocity
                Rigidbody.angularVelocity = angularVel;

                // rotate rigidbody's velocity as well to generate immediate velocity redirection
                // manual velocity steering coefficient
                float velocitySteering = 25.0f;//0.4f / Mathf.Deg2Rad;

                // rotate our velocity based on current steer value
                Rigidbody.velocity = Quaternion.AngleAxis(turningPower * Mathf.Sign(localVel.z) * velocitySteering * m_CurrentGrip * Time.fixedDeltaTime, transform.up) * Rigidbody.velocity;
            }
            else
            {
                m_InAir = true;
            }

            bool validPosition = false;
            if (Physics.Raycast(transform.position + (transform.up * 0.1f), -transform.up, out RaycastHit hit, 3.0f, 1 << 9 | 1 << 10 | 1 << 11)) // Layer: ground (9) / Environment(10) / Track (11)
            {
                Vector3 lerpVector = (m_HasCollision && m_LastCollisionNormal.y > hit.normal.y) ? m_LastCollisionNormal : hit.normal;
                m_VerticalReference = Vector3.Slerp(m_VerticalReference, lerpVector, Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime * (GroundPercent > 0.0f ? 10.0f : 1.0f)));    // Blend faster if on ground
            }
            else
            {
                Vector3 lerpVector = (m_HasCollision && m_LastCollisionNormal.y > 0.0f) ? m_LastCollisionNormal : Vector3.up;
                m_VerticalReference = Vector3.Slerp(m_VerticalReference, lerpVector, Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime));
            }

            validPosition = GroundPercent > 0.7f && !m_HasCollision && Vector3.Dot(m_VerticalReference, Vector3.up) > 0.9f;

            // Airborne / Half on ground management
            if (GroundPercent < 0.7f)
            {
                Rigidbody.angularVelocity = new Vector3(0.0f, Rigidbody.angularVelocity.y * 0.98f, 0.0f);
                Vector3 finalOrientationDirection = Vector3.ProjectOnPlane(transform.forward, m_VerticalReference);
                finalOrientationDirection.Normalize();
                if (finalOrientationDirection.sqrMagnitude > 0.0f)
                {
                    Rigidbody.MoveRotation(Quaternion.Lerp(Rigidbody.rotation, Quaternion.LookRotation(finalOrientationDirection, m_VerticalReference), Mathf.Clamp01(AirborneReorientationCoefficient * Time.fixedDeltaTime)));
                }
            }
            else if (validPosition)
            {
                m_LastValidPosition = transform.position;
                m_LastValidRotation.eulerAngles = new Vector3(0.0f, transform.rotation.y, 0.0f);
            }
        }

        public float getMaxAngularVelocity()
        {
            float angularVelocitySteering = 0.4f;
            float turningPower =  m_FinalStats.Steer;
            return turningPower * angularVelocitySteering;
        }

        public float getMaxLateralGs()
        {
            return getMaxLateralGsForWear(TireWearProportion());
        }

        public float getMaxLateralGsForWear(float tireWear)
        {
            return (1 - tireWear) * (m_FinalStats.MaxGs - m_FinalStats.MinGs) + m_FinalStats.MinGs;
        }

        public float getTurningRadius()
        {
            
            var output = (Vector3.Dot(Rigidbody.velocity, Rigidbody.transform.forward) / Rigidbody.angularVelocity.y);
            if (float.IsInfinity(output) || float.IsNaN(output))
                return 1000;
            return output;
        }

        public float getMaxSpeedForState()
        {
            return getMaxSpeedForRadiusAndWear(getTurningRadius(), TireWearProportion());
        }

        public float getMaxSpeedForRadiusAndWear(float radius, float tireWear)
        {
            if (radius == 0)
                return m_FinalStats.TopSpeed;
            var maxAllowedSpeed = Mathf.Sqrt(getMaxLateralGsForWear(tireWear) * 9.81f * (Mathf.Abs(radius)));
            // print(maxAllowedSpeed);
            if ((float.IsInfinity(maxAllowedSpeed) || float.IsNaN(maxAllowedSpeed)))
            {
                maxAllowedSpeed = m_FinalStats.TopSpeed;
            }
            return Mathf.Clamp(maxAllowedSpeed, 0.0001f, m_FinalStats.TopSpeed);
        }

        public Vector3 ForwardDirection()
        {
            return transform.InverseTransformVector(Rigidbody.velocity).normalized;
        }

        public Vector3 LeftDirection()
        {
            return Quaternion.Euler(0, 90, 0) * ForwardDirection();
        }

        public Vector3 RightDirection()
        {
            return Quaternion.Euler(0, -90, 0) * ForwardDirection();
        }
    }
}
