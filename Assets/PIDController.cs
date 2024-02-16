using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDController
{
    private float integral;
    private float previousError;

    public float Update(float error, float delta, float kP, float kI, float kD)
    {
        integral += error * delta;
        float derivative = (error - previousError) / delta;

        float correction = kP * error + kI * integral + kD * derivative;

        previousError = error;

        return correction;
    }
}
