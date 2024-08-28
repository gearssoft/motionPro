using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoxPlane : MonoBehaviour
{
    static Vector2 ScreenSize = new(10, 10);

    //일정시간 사용되지않으면 지우기
    public float lastUpdateTime = 0.0f;

    public static void SetScreenSize(Vector2 size)
    {
        ScreenSize *= size;
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        //3초이상 사용되지않으면 지우기
        if (Time.time - lastUpdateTime > 3.0f)
        {
            //destroy this object
            // Destroy(gameObject);
            gameObject.SetActive(false);
        }

    }
    public void SetPosition(Vector2 topleft, Vector2 bottomRight)
    {
        if(gameObject.activeSelf == false)
        {
            gameObject.SetActive(true);
        }

        //update last update time
        lastUpdateTime = Time.time;
        // Normalize coordinates to 3D space (0 to 2 range)
        Vector2 topLeft3D = topleft;
        Vector2 bottomRight3D = bottomRight;

        // Calculate the center position of the plane
        Vector2 center = (topLeft3D + bottomRight3D) / 2;

        center *= ScreenSize;

        // Calculate the size of the plane
        Vector2 size = bottomRight3D - topLeft3D;

        // Update the position and scale of the plane
        transform.localPosition = (Vector3)center;
        size *= (ScreenSize/10);
        transform.localScale = new Vector3(size.x, 1.0f, size.y);
        
    }

}
