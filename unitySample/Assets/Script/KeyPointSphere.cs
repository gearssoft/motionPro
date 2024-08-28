using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyPointSphere : MonoBehaviour
{
    public int index = 0;

    static Vector2 ScreenSize = new(10, 10);

    //일정시간 사용되지않으면 지우기
    public float lastUpdateTime = 0.0f;

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
    public void SetPosition(Vector2 center)
    {
        //update last update time
        lastUpdateTime = Time.time;

        // if center is zero then hide the sphere
        if (center == Vector2.zero)
        {
            gameObject.SetActive(false);
            return;
        }
        else
        {
            gameObject.SetActive(true);
        }
        // Update the position and scale of the plane
        center *= ScreenSize;
        transform.localPosition = (Vector3)center;
    }

    public static void SetScreenSize(Vector2 size)
    {
        ScreenSize *= size;
    }
}
