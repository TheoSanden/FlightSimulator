using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class MathUtils 
{
    /// <summary>
    /// remaps a value from range 1 to range 2, for.ex 200 within the range of 100-300 = 0.5 in the range of 0-1
    /// </summary>
    /// <returns>The remapped value</returns>
    public static float remapRange(float value, float low1, float high1, float low2, float high2)
    {
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    }
}
