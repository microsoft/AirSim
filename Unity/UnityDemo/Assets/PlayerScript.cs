
using AirSimUnity;
using UnityEngine;
using UnityEngine.Networking;


public class PlayerScript : NetworkBehaviour
{
    [SerializeField]
    Behaviour[] componentsToDisabled;
    Camera sceneCamera;
    
    private void Start()
    {
         var drone = GetComponent<Drone>();
        if (!isLocalPlayer)
        {
            for (int i = 0; i < componentsToDisabled.Length; i++)
            {
                componentsToDisabled[i].enabled = false;
            }
        }
        else
        {

            drone.enabled = true;
            for (int i = 0; i < componentsToDisabled.Length; i++)
            {
                componentsToDisabled[i].enabled = true;
            }


            GameObject viewCameras = GameObject.FindGameObjectWithTag("HUD");
            var hud = viewCameras.GetComponent<AirSimHUDScript>();
            hud.SetUpViewCameras();
            sceneCamera = Camera.main;
            if (sceneCamera != null)
            {
                sceneCamera.gameObject.SetActive(false);
            }
        }
    }




    private void OnDisable()
    {
        if (sceneCamera != null)
        {
            sceneCamera.gameObject.SetActive(true);
        }
    }





}
