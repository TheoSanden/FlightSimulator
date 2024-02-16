using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotor : MonoBehaviour 
{
    private float revolutionsPerSecond;
    private float maxRevolutionsPerSecond = 20;
    private float thurstToRotorSpeedConversion = 600000;
    private float decleration = 1;

    [SerializeField]private AnimationCurve rotorCurve;

    public void UpdateRotorSpeed(float thrust) 
    {
        revolutionsPerSecond = rotorCurve.Evaluate(Mathf.Clamp(thrust / thurstToRotorSpeedConversion, 0, 1)) * maxRevolutionsPerSecond;
    }
    void Update() 
    {
        transform.Rotate(Vector3.forward,revolutionsPerSecond * 360 * Time.deltaTime);
        revolutionsPerSecond = (revolutionsPerSecond - Time.deltaTime * decleration <0) ? 0: Time.deltaTime * decleration;
        
    }
}
