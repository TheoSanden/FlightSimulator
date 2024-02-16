using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrossHair : MonoBehaviour {
    private Camera camera;
    [SerializeField] private float centerRadius = 10f;
    [SerializeField] private float estimatedScreenRadius;
    [SerializeField] private EasingFunction.Ease centerToEdgeEasing;
    private EasingFunction.Function centerToEdgeEasingFuntion;

    private Vector2 canvasScale;

    private Vector3 cursorStartPosition;

    private RectTransform rect;
    // Start is called before the first frame update
    void Start() 
    {
        camera = Camera.main;
        centerToEdgeEasingFuntion = EasingFunction.GetEasingFunction(centerToEdgeEasing);

        Rect parentRect = GetComponentInParent<RectTransform>().rect;
        canvasScale = new Vector2(parentRect.width, parentRect.height);

        rect = this.GetComponent<RectTransform>();
        cursorStartPosition = this.rect.localPosition;
        cursorStartPosition.z = 0;

        estimatedScreenRadius = Screen.height/2;

        Cursor.lockState = CursorLockMode.Confined;
        Cursor.visible = false;
    }

    // Update is called once per frame
    void Update()
    {
        UpdateCrossHairPosition();       
    }

    public Vector3 GetEasedMouseDirection() 
    {
        Vector3 screenOrigo = new Vector3(Screen.width / 2, Screen.height / 2);
        
        Vector3 mouseDirection = Input.mousePosition - screenOrigo;
        
        mouseDirection.x /= (Screen.width/2);
        mouseDirection.y /= (Screen.height/2);
        
        float easedX = centerToEdgeEasingFuntion(0, 1, Mathf.Abs(mouseDirection.x));
        float easedY = centerToEdgeEasingFuntion(0, 1, Mathf.Abs(mouseDirection.y));

        return new Vector3(easedX * Mathf.Sign(mouseDirection.x),easedY * Mathf.Sign(mouseDirection.y));
    }

    void UpdateCrossHairPosition() {
        Vector3 screenOrigo = new Vector3(Screen.width / 2, Screen.height / 2);
        
        Vector3 mouseDirection = Input.mousePosition - screenOrigo;
        
        mouseDirection.x /= (Screen.width/2);
        mouseDirection.y /= (Screen.height/2);
        
        float easedX = centerToEdgeEasingFuntion(0, 1, Mathf.Abs(mouseDirection.x));
        float easedY = centerToEdgeEasingFuntion(0, 1, Mathf.Abs(mouseDirection.y));

        
        Vector3 newPosition = cursorStartPosition + new Vector3(centerRadius * easedX * Mathf.Sign(mouseDirection.x), centerRadius * easedY * Mathf.Sign(mouseDirection.y));
        newPosition.z = 0;
        
        
        rect.localPosition = newPosition;
    }
}
