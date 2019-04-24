using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using AirSimUnity;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public class InitializeAirSim : MonoBehaviour
{
    private static string AIRSIM_DATA_FOLDER = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/AirSim/";

    void Awake()
    {
        if (!Directory.Exists(AIRSIM_DATA_FOLDER))
        {
            Directory.CreateDirectory(AIRSIM_DATA_FOLDER);
        }

        AirSimSettings.Initialize();

        var simMode = AirSimSettings.GetSettings().SimMode;
        if (simMode != "")
        {
            switch (simMode)
            {
                case "Car":
                case "Multirotor":
                case "ComputerVision":
                    LoadSceneAsPerSimMode(simMode);
                    break;
                default:
                    Debug.Log("Notice: Unknown SimMode specified in 'settings.json' file.");
                    break;
            }
        }
    }

    public void LoadSceneAsPerSimMode(string load_name)
    {
        if (load_name == "Car")
            AirSimSettings.GetSettings().SimMode = "Car";
        else if (load_name == "Multirotor")
            AirSimSettings.GetSettings().SimMode = "Multirotor";

            
        // Once SimMode is known we make final adjustments and check of settings based on the mode selected.
        if (AirSimSettings.GetSettings().ValidateSettingsForSimMode())
        {
            if (load_name == "Car")
            {
                SceneManager.LoadSceneAsync("Scenes/CarDemo", LoadSceneMode.Single);
            }
            else if (load_name == "Multirotor")
            {
                SceneManager.LoadSceneAsync("Scenes/DroneDemo", LoadSceneMode.Single);
            }
        }
        else
        {
            Debug.LogError("Error: Had a problem with the 'settings.json' file for SimMode selected '" + load_name + "'.");
#if UNITY_EDITOR
            EditorUtility.DisplayDialog("Had a problem with the 'settings.json' file for SimMode selected: '" + load_name + "'.",
                "Had a problem with the 'settings.json' file for SimMode selected: '" + load_name + "'.",
                "Exit");
#endif
            Application.Quit();
        }
    }

}

