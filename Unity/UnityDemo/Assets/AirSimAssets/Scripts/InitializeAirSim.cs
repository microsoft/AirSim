using System;
using System.IO;
using AirSimUnity;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public class InitializeAirSim : MonoBehaviour
{
    void Awake()
    {
        if (GetAirSimSettingsFileName() != string.Empty)
        {
            if (AirSimSettings.Initialize())
            {
                if (AirSimSettings.GetSettings().SimMode == "")
                {
                    if (Application.isEditor)
                    {
                        var option = EditorUtility.DisplayDialogComplex("SimMode is not specified in Settings.json!",
                            "Please select desired SimMode as per loaded scene.",
                            "Car", "Exit", "Multirotor");

                        switch (option)
                        {
                            case 2:
                                AirSimSettings.GetSettings().SimMode = "Multirotor";
                                break;
                            case 1:
                                EditorApplication.Exit(1);
                                break;
                            case 0:
                                AirSimSettings.GetSettings().SimMode = "Car";
                                break;
                        }
                    }
                    else
                    {
                        // Default to Car when Sim mode is missing.
                        Debug.LogError("Sim mode is missing, defaulting to the Car demo");
                        AirSimSettings.GetSettings().SimMode = "Car";
                    }
                    
                }
                LoadSceneAsPerSimMode();
            }
        }
        else
        {
            Debug.LogError("'Settings.json' file either not present or not configured properly.");
            if (Application.isEditor) {
                EditorUtility.DisplayDialog("Missing 'Settings.json' file!!!", "'Settings.json' file either not present or not configured properly.", "Exit");
            }
            Application.Quit();
        }
    }

    public static string GetAirSimSettingsFileName()
    {
        string fileName = Application.dataPath + "\\..\\settings.json";
        if (File.Exists(fileName))
        {
            return fileName;
        }

        fileName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), Path.Combine("AirSim", "settings.json"));
        if (File.Exists(fileName))
        {
            return fileName;
        }

        if (CreateSettingsFileWithDefaultValues(fileName))
            return fileName;
        else
            return string.Empty;
    }

    public static bool CreateSettingsFileWithDefaultValues(string fileName)
    {
        var result = false;
        try
        {
            string content = "{\n \"SimMode\" : \"\", \n \"SettingsVersion\" : 1.2, \n \"SeeDocsAt\" : \"https://github.com/Microsoft/AirSim/blob/master/docs/settings.md\"\n}";
            //settings file created at Documents\AirSim with name "setting.json".
            StreamWriter writer = new StreamWriter(File.Open(fileName, FileMode.OpenOrCreate, FileAccess.Write));
            writer.WriteLine(content);
            writer.Close();
            result = true;
        }
        catch (Exception ex)
        {
            Debug.LogError("Unable to create settings.json file @ " + fileName + " Error :- " + ex.Message);
            result = false;
        }
        return result;
    }

    private void LoadSceneAsPerSimMode()
    {
        if (AirSimSettings.GetSettings().SimMode == "Car")
            SceneManager.LoadSceneAsync("Scenes/CarDemo", LoadSceneMode.Single);
        else if (AirSimSettings.GetSettings().SimMode == "Multirotor")
            SceneManager.LoadSceneAsync("Scenes/DroneDemo", LoadSceneMode.Single);
    }
}
