using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;

public class MovieController : MonoBehaviour
{
    public PlayerSetting m_PlayerSetting;

    public VideoPlayer m_VideoPlayer;
    public VideoClip[] m_VideoClips;
    public bool m_IsPlaying = false;
    public bool m_IsLoop = false;
    public bool m_IsStarting = false;
    public int m_Index = 0;


    public float m_Time = 0.5f;
    public float m_FadeTime = 1.0f;
    public Material m_Material;

    // Start is called before the first frame update
    void Start()
    {
        m_Time = 1.0f;
        m_IsStarting = false;

        m_VideoPlayer.clip = m_VideoClips[m_Index]; 
        m_VideoPlayer.Play();
        m_IsPlaying = true;

        if ( m_Index == 0 )
        {
            m_IsLoop = true;
            m_VideoPlayer.isLooping = true;
        }

        m_VideoPlayer.loopPointReached += EndReached;
    }

    // Update is called once per frame
    void Update()
    {
        if ( Input.GetKeyDown( KeyCode.Space ) )
        {
            if ( m_VideoPlayer.isPlaying )
            {
                m_VideoPlayer.Pause();

                m_IsPlaying = false;
            }
            else
            {
                m_VideoPlayer.Play();
                
                m_IsPlaying = true;
            }
        }

        // if ( Input.GetKeyDown( KeyCode.S ) )
        // {
        //     StartCoroutine( Fade() );
        // }

        if ( Input.GetKeyDown( KeyCode.A ) )
        {
            if ( m_Index > -1 )
            {
                StartCoroutine( Fade() );
                m_Index++;
                if ( m_Index >= m_VideoClips.Length )
                {
                    m_Index = 0;
                    if ( !m_IsLoop )
                    {
                        m_VideoPlayer.isLooping = true;
                        m_IsLoop = true;
                    }
                }

                m_VideoPlayer.clip = m_VideoClips[m_Index];
                m_VideoPlayer.Play();

                if ( m_IsLoop && m_Index != 0 )
                {
                    m_IsLoop = false;
                    m_VideoPlayer.isLooping = false;
                }
            }
        }


        if ( m_IsStarting )
        {
            Debug.Log( "시작" );
            m_IsStarting = false;

            if ( m_Index == 0 )
            {
                StartCoroutine( Fade() );
                m_Index++;

                m_VideoPlayer.clip = m_VideoClips[m_Index];
                m_VideoPlayer.Play();

                if ( m_IsLoop && m_Index != 0 )
                {
                    m_IsLoop = false;
                    m_VideoPlayer.isLooping = false;
                }
            }
        }

    }

    void EndReached( VideoPlayer vp )
    {
        if ( m_Index ==0 )
        {
            
        }
        else
        {
            StartCoroutine( Fade() );
            m_Index++;
            if ( m_Index >= m_VideoClips.Length )
            {
                m_Index = 0;

                // m_IsStarting = true;

                if ( !m_IsLoop )
                {
                    m_VideoPlayer.isLooping = true;
                    m_IsLoop = true;
                }   
            }

            m_VideoPlayer.clip = m_VideoClips[m_Index];
            m_VideoPlayer.Play();
        }
    }

    public IEnumerator Fade( )
    {
        Color color = m_Material.color;

        while ( color.r > 0.0f )
        {
            m_Time += Time.deltaTime / m_FadeTime;
            color.r = Mathf.Lerp( 1, 0, m_Time );
            color.g = Mathf.Lerp( 1, 0, m_Time );
            color.b = Mathf.Lerp( 1, 0, m_Time );
            m_Material.color = color;
            yield return null;
        }

        m_Time = 0f;

        yield return new WaitForSeconds( 0.5f );

        while ( color.r < 1.0f )
        {
            m_Time += Time.deltaTime / m_FadeTime;
            color.r = Mathf.Lerp( 0, 1, m_Time );
            color.g = Mathf.Lerp( 0, 1, m_Time );
            color.b = Mathf.Lerp( 0, 1, m_Time );
            m_Material.color = color;
            yield return null;
        }

        m_Time = 1.0f;
    }

}
