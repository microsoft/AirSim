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
                if (AirSimSettings.GetSettings().SimMode != "")
                {
                    LoadSceneAsPerSimMode(AirSimSettings.GetSettings().SimMode);
                }
            }
        }
        else
        {
#if UNITY_EDITOR
            EditorUtility.DisplayDialog("Missing 'Settings.json' file!!!", "'Settings.json' file either not present or not configured properly.", "Exit");
            EditorApplication.Exit(1);
#else
            Application.Quit();
#endif
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

    public void LoadSceneAsPerSimMode(string load_name)
    {
        Debug.Log(load_name);
        if (load_name == "Car")
            SceneManager.LoadSceneAsync("Scenes/CarDemo", LoadSceneMode.Single);
        else if (load_name == "Multirotor")
            SceneManager.LoadSceneAsync("Scenes/DroneDemo", LoadSceneMode.Single);
    }
}
