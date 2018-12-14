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
			if(AirSimSettings.Initialize())
			{

                switch (AirSimSettings.GetSettings().SimMode)
                {
                    case "Car":
                    {
                        LoadSceneAsPerSimMode(AirSimSettings.GetSettings().SimMode);
                        break;
                    }
                    case "Multirotor":
                    {
                        LoadSceneAsPerSimMode(AirSimSettings.GetSettings().SimMode);
                        break;
                    }
				}
            }
        }
        else
        {
            Debug.LogError("'Settings.json' file either not present or not configured properly.");
#if UNITY_EDITOR
                EditorUtility.DisplayDialog("Missing 'Settings.json' file!!!", "'Settings.json' file either not present or not configured properly.", "Exit");
#endif
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
        string linuxFileName = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), Path.Combine("Documents/AirSim", "settings.json"));
        if (File.Exists(fileName))
        {
            return fileName;
        }
        else if (File.Exists(linuxFileName))
        {
            return linuxFileName;
        }
        if (CreateSettingsFileWithDefaultValues(fileName))
            return fileName;
        else if (CreateSettingsFileWithDefaultValues(linuxFileName))
            return linuxFileName;
        else
            return string.Empty;
    }

    public static bool CreateSettingsFileWithDefaultValues(string fileName)
    {
        var result = false;
        try
        {
            if (fileName.Substring(0, 5) == "/home")
                Directory.CreateDirectory(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Documents/AirSim"));
            else
                Directory.CreateDirectory(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "AirSim"));

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
        if (load_name == "Car")
        {
            AirSimSettings.GetSettings().SimMode = "Car";
            SceneManager.LoadSceneAsync("Scenes/CarDemo", LoadSceneMode.Single);
        }
        else if (load_name == "Multirotor")
        {
            AirSimSettings.GetSettings().SimMode = "Multirotor";
            SceneManager.LoadSceneAsync("Scenes/DroneDemo", LoadSceneMode.Single);
        }
    }
}
