using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class DebugUI : MonoBehaviour 
{
    [SerializeField] private Engine engine;
    [SerializeField] private TMP_Text dragText;
    [SerializeField] private TMP_Text thrustText;
    [SerializeField] private TMP_Text velocityText;
    // Start is called before the first frame update

    // Update is called once per frame
    void Update() 
    {  //dragText.text = "Drag: " + (int)engine.GetDrag().magnitude;
       thrustText.text = "Thrust: " + (int)engine.GetThrust().magnitude;
       velocityText.text = "Velocity: " + (int)(engine.rigidBody.velocity.magnitude * 3600) / 10000 + " Km/h";
    }
}
