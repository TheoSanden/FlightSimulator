using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Engine : MonoBehaviour {
    private float maxVelocityCap = 550;
    [SerializeField] private Vector3 COM;
    
    private Rotor rotor;
    [HideInInspector]public Rigidbody rigidBody;
    
    [SerializeField] private GameObject RightAileron;
    [SerializeField] private GameObject LeftAileron;
    
    [Header("Thrust")]
    [SerializeField] private float forwardsThrust = 0;
    [SerializeField] private float acceleration = 10;
    [SerializeField] private float deceleration = 10;
    [SerializeField] private float maxThrust = 100;
    

    [Header("Drag")]
    [SerializeField] private float dragCoefficient = 1;
    [SerializeField] private float aileronDragCoefficient = 1;
    [SerializeField] private float inducedDrag = 1;
    [SerializeField] private AnimationCurve dragCurve;
    
    [Header("Lift")]
    [SerializeField] private float liftPower = 10;
    [SerializeField] private float aileronLiftPower = 2;
    [SerializeField] private AnimationCurve AOA;
    private Vector3  AilerionLerpVector = Vector3.zero;

    [Header("Steering")] 
    [SerializeField] private Vector3 turnSpeed;
    [SerializeField] private Vector3 turnAcceleration;
    [SerializeField] private AnimationCurve steeringCurve;
    /// <summary>
    /// Makes it easier to turn in case your plane is turned in a different direction from the local velocity
    /// </summary>
    [SerializeField] private float reverseMultiplier = 1;

    [Header("Upright Correction PID values")] 
    [SerializeField] private float KP = 0.1f;
    [SerializeField] private float KI = 0.01f;
    [SerializeField] private float KD = 0.01f;
    [SerializeField] private float correctionArea = 1;
        
    [Header("Debug")] 
    [SerializeField] private Vector3 AileronOffset;
    [SerializeField] private Vector2 AileronSize;
     private Vector3 RightAileronRotation;
     private Vector3 LeftAileronRotation;
    [SerializeField] private Vector2 lowerAileronRotationConstraints;
    [SerializeField] private Vector2 upperAileronRotationConstraints;
    [SerializeField] private Vector3 BackFinPosition;
    [SerializeField] private Vector3 BackFinRotation;
    [SerializeField] private Vector3 BackFinSize;

    [SerializeField] private CrossHair crosshair;

    private Vector3 localVelocity;
    private Vector3 localAngularVelocity;

    void Start() 
    {
        rigidBody = this.GetComponent<Rigidbody>();
        rotor = GetComponentInChildren<Rotor>();
        
        if (!rotor) 
        {
            Debug.Log("RotorNotFound");
        }    
    }
    
    #if UNITY_EDITOR
    private void OnDrawGizmosSelected() 
    {
        Gizmos.color = Color.blue;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.DrawSphere(COM, 1.0f);

        Handles.color = Color.yellow;
        
        DrawAileron(new Vector3(AileronOffset.x,AileronOffset.y,AileronOffset.z),RightAileronRotation,AileronSize);
        DrawAileron(new Vector3(-AileronOffset.x,AileronOffset.y,AileronOffset.z),LeftAileronRotation,AileronSize);
        DrawAileron(BackFinPosition,BackFinRotation,BackFinSize);
    }
    
    
    private void DrawAileron(Vector3 fromPosition, Vector3 rotation, Vector2 size) {


        Handles.matrix = transform.localToWorldMatrix;
         
         Matrix4x4 rotationMatrix = Matrix4x4.Rotate(Quaternion.Euler(rotation));

         Vector3[] planePoints = new[] 
         {
             new Vector3(-size.x, 0, 0), 
             new Vector3(size.x, 0, 0),
             new Vector3(size.x, 0, -size.y * 2),
             new Vector3(-size.x, 0, -size.y * 2),
             new Vector3(-size.x, 0, 0)
         };

         for(int i = 0; i < planePoints.Length; i++) 
         {
             planePoints[i] = rotationMatrix.MultiplyPoint3x4(planePoints[i]);
             planePoints[i] += fromPosition;
         }
         
         Handles.DrawAAConvexPolygon(planePoints);
    }
    #endif
    public Vector3 GetDrag(Vector3 upVectorOfObject,float dragCoefficient) 
    {
        var lv = localVelocity;
        var lv2 = lv.sqrMagnitude;

        var speed = Mathf.Max(0, localVelocity.z);
        var speedModifier = dragCurve.Evaluate(speed / maxVelocityCap);
        
        float dragDot = Mathf.Abs(Vector3.Dot(upVectorOfObject, localVelocity.normalized));
        dragDot = Mathf.Abs(dragDot);

        var drag = dragDot * speedModifier * dragCoefficient * (lv2/2) * -lv.normalized;
        return drag;
    }
    //not done, the drag coefficient should be the same for if the wings are up or down
    public Vector3 GetAileronDragCoefficient(GameObject aileron) 
    {
        Vector3 velocity = rigidBody.velocity;
        float dragDot = Mathf.Abs(Vector3.Dot(velocity.normalized, -aileron.transform.forward));
        float angleCoefficient = MathUtils.remapRange(dragDot, 1, 0, 0, 1);
        angleCoefficient = Mathf.Abs(angleCoefficient);
        
        return  angleCoefficient *  Mathf.Pow(velocity.magnitude,2) * aileronDragCoefficient * -velocity.normalized;
    }
    float GetReverseMultiplier() 
    {
        float dot = Vector3.Dot(localVelocity.normalized, Vector3.forward);
        float remappedDot = MathUtils.remapRange(dot, -1, 1, reverseMultiplier, 1);
        
        //could add a curve to this here.

        return remappedDot;
    }

    public Vector3 CalculateCompoundedLift() 
    {
        
        Vector3 planeLift = CalculateLift(transform.rotation.x / 360, Vector3.right,liftPower,AOA);
        Vector3 leftAileronLift = CalculateLift(-LeftAileron.transform.rotation.x/360, Vector3.right, aileronLiftPower, AOA);
        Vector3 rightAileronLift = CalculateLift(-RightAileron.transform.rotation.x/360, Vector3.right, aileronLiftPower, AOA);
        //Vector3 verticalStabilizerLift = CalculateLift(transform.rotation.z/360, Vector3.up, aileronLiftPower, AOA);
        
        Vector3 lift = planeLift + leftAileronLift + rightAileronLift;
        
        return lift;
    }


    
    Vector3 CalculateLift(float easedAngleOfAttack, Vector3 staticRightAxis, float liftPower, AnimationCurve aoaCurve)
    {
        Vector3 airFlow = Vector3.ProjectOnPlane(localVelocity, staticRightAxis);
        float v2 = airFlow.sqrMagnitude;

        float liftCoefficient = aoaCurve.Evaluate(easedAngleOfAttack);
        float liftForce = v2 * liftCoefficient * liftPower;

        Vector3 liftDirection = Vector3.Cross(airFlow.normalized, staticRightAxis);
        Vector3 lift = liftDirection * liftForce;

        float dragForce = Mathf.Pow(liftCoefficient, 2) * this.inducedDrag;
        Vector3 dragDirection = -liftDirection.normalized;
        Vector3 inducedDrag = dragDirection * (v2 * dragForce);

        return lift + inducedDrag;
    }

    private void UpdateLocalParameters() 
    {
        Quaternion invRotation = Quaternion.Inverse(rigidBody.rotation);
        localVelocity = invRotation * rigidBody.velocity;
        localAngularVelocity = invRotation * rigidBody.angularVelocity;
    }



    void ApplyTorque(float delta) 
    {
        float speed = Mathf.Max(0, localVelocity.z);
        float steeringPower = steeringCurve.Evaluate(speed/maxVelocityCap);
        
        Vector3 mouseDirection = crosshair.GetEasedMouseDirection();
        mouseDirection.x *= turnSpeed.z * steeringPower;
        mouseDirection.y *= turnSpeed.x * steeringPower;
        Vector3 av = localAngularVelocity * Mathf.Rad2Deg;
        
        Debug.Log(mouseDirection);
        
        var correction = new Vector3(
            CalculateSteering(delta, av.x, -mouseDirection.y, turnAcceleration.x * steeringPower),
            CalculateSteering(delta, av.y,  mouseDirection.x,turnAcceleration.y * steeringPower),
             CalculateSteering(delta, av.z, -mouseDirection.x, turnAcceleration.z * steeringPower));
        //I removed this and decreased the jaw acceleration instead, works like a charm
        //s( Mathf.Abs(crosshair.GetEasedMouseDirection().magnitude) < correctionArea && Mathf.Abs(crosshair.GetEasedMouseDirection().y) < correctionArea )? CalculateUprightCorrection(delta,turnAcceleration.z * steeringPower,KP,KI,KD):
        rigidBody.AddRelativeTorque(correction * Mathf.Deg2Rad,ForceMode.VelocityChange);
    }
    float CalculateSteering(float delta, float angularVelocity, float targetVelocity, float acceleration) 
    {
        float error = targetVelocity - angularVelocity;
        float deltaAcceleration = acceleration * delta;

        return Mathf.Clamp(error, -deltaAcceleration, deltaAcceleration);
    }

    private PIDController uprightCorrectionPid = new PIDController();
    float CalculateUprightCorrection(float delta, float acceleration, float kP, float kI, float kD) 
    {
        float error = Mathf.DeltaAngle(transform.eulerAngles.z, 0);
        float deltaAcceleration = acceleration * delta;
        float pidCorrection = uprightCorrectionPid.Update(error, delta, kP, kI, kD);
        return Mathf.Clamp(pidCorrection , -deltaAcceleration, deltaAcceleration);
    }

    private void RevEngine(float delta)
    {
        forwardsThrust = forwardsThrust + delta * acceleration > maxThrust
            ? maxThrust
            : forwardsThrust + delta * acceleration;
        
        rotor.UpdateRotorSpeed(forwardsThrust);
    }

    private void Decelerate(float delta) 
    {
        forwardsThrust = forwardsThrust - delta * acceleration < 0
            ? 0
            : forwardsThrust - delta * deceleration;
        rotor.UpdateRotorSpeed(forwardsThrust);
    }

    private void Update()
    {
        if (Input.GetKey(KeyCode.Space)) 
        {
            RevEngine(Time.deltaTime);
        }
        else 
        {
            Decelerate(Time.deltaTime);
        }
        Debug.DrawLine(transform.position, transform.position + rigidBody.velocity,Color.cyan,Time.deltaTime);
        Debug.DrawLine(transform.position, transform.position + transform.forward * 100, Color.green,Time.deltaTime);
        Debug.DrawLine(transform.position, transform.position + CalculateCompoundedLift(),Color.yellow, Time.deltaTime);
        Debug.DrawLine(transform.position,transform.position + GetDrag(transform.up,dragCoefficient)  + GetDrag(LeftAileron.transform.up,aileronDragCoefficient) + GetDrag(RightAileron.transform.up, aileronDragCoefficient),Color.magenta, Time.deltaTime);
    }
    void FixedUpdate() 
    {
        UpdateLocalParameters();
        UpdateAileronRotation();
        ApplyTorque(Time.fixedDeltaTime);
        rigidBody.AddRelativeForce(GetThrust() + GetDrag(transform.up,dragCoefficient)  + GetDrag(LeftAileron.transform.up,aileronDragCoefficient) + GetDrag(RightAileron.transform.up, aileronDragCoefficient) + CalculateCompoundedLift());
    }
    
    public Vector3 GetThrust() 
    {
        return Vector3.forward * forwardsThrust * GetReverseMultiplier();
    }
    
    //right now pulling the mouse up results in only the ailerons being half way down
    private void UpdateAileronRotation() 
    {
        Vector3 mouseDirection = crosshair.GetEasedMouseDirection();
        
        float leftAileronDirectionalInfluence = (-mouseDirection.x - mouseDirection.y) / 2;
        float rightAileronDirectionalInfluence = (mouseDirection.x - mouseDirection.y) / 2;
        
        Vector3 newLeftAileronRotation = Vector3.right * MathUtils.remapRange(leftAileronDirectionalInfluence, -1,1,lowerAileronRotationConstraints.x ,upperAileronRotationConstraints.x);
        Vector3 newRightAileronRotation = Vector3.right * MathUtils.remapRange(rightAileronDirectionalInfluence, -1,1,lowerAileronRotationConstraints.x ,upperAileronRotationConstraints.x);
        
        UpdateAileronRotationAndPosition(LeftAileron,new Vector3(-AileronOffset.x,AileronOffset.y,AileronOffset.z),newLeftAileronRotation);
        UpdateAileronRotationAndPosition(RightAileron,new Vector3(AileronOffset.x,AileronOffset.y,AileronOffset.z),newRightAileronRotation);
    }
   /* 
    //Just Temporary
    private void SetAileronAngle(int direction) 
    {
        UpdateAileronRotationAndPosition(LeftAileron,new Vector3(-AileronOffset.x,AileronOffset.y,AileronOffset.z),new Vector3(45 * direction,0,0));
        UpdateAileronRotationAndPosition(RightAileron,new Vector3(AileronOffset.x,AileronOffset.y,AileronOffset.z),new Vector3(45 * direction,0,0));
    }*/

    [ContextMenu("UpdateAileronTransform")]
    void UpdateAileronTransform()
    {
        Vector3 aileronSize = new Vector3(AileronSize.x * 2, 0.2f, AileronSize.y * 2);
        LeftAileron.transform.localScale = aileronSize;
        RightAileron.transform.localScale = aileronSize;
        
        UpdateAileronRotationAndPosition(LeftAileron,new Vector3(-AileronOffset.x,AileronOffset.y,AileronOffset.z),LeftAileronRotation);
        UpdateAileronRotationAndPosition(RightAileron,new Vector3(AileronOffset.x,AileronOffset.y,AileronOffset.z),RightAileronRotation);
    }

    void UpdateAileronRotationAndPosition(GameObject aileron,Vector3 localPosition,Vector3 rotation)
    {
        Quaternion localRotation = Quaternion.Euler(rotation);
        //might need to reverse the rotations
        Quaternion planeRelativeRotation = transform.rotation * localRotation;

        aileron.transform.rotation = planeRelativeRotation;
        

        Vector3 localPositionOffset = Vector3.back * AileronSize.y;

        localPositionOffset = planeRelativeRotation * localPositionOffset;

        Vector3 worlPosition = transform.position + transform.rotation * localPosition + localPositionOffset;

        aileron.transform.position = worlPosition;
        /*
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(Quaternion.Euler(rotation));

        aileron.transform.localRotation = Quaternion.Euler(rotation);

        rotationMatrix *= Matrix4x4.Rotate(transform.rotation);


        Vector3 localOffset = Vector3.back * aileron.transform.localScale.z/2 - Vector3.up * 0.18f;
        localOffset = rotationMatrix.MultiplyPoint3x4(localOffset);

        localOffset += position;

        aileron.transform.localPosition = localOffset;

        //For debugPurpouses
        Vector3 attachedWorldPosition = transform.localToWorldMatrix.MultiplyPoint3x4(position);
        Vector3 aileronDirection =  transform.rotation * Quaternion.Euler(rotation) * Vector3.back * aileron.transform.localScale.z/2 - Vector3.up * 0.18f;
        Debug.DrawLine(attachedWorldPosition,attachedWorldPosition + aileronDirection,Color.black,Time.fixedDeltaTime);
        //Debug.DrawLine(aileron.transform.position, aileron.transform.position + aileron.transform.localPosition.normalized*2,Color.black,Time.fixedDeltaTime);
        */
    }
    
}