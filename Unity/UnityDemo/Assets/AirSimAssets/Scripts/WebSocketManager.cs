
using CI.HttpClient;
using System;
using System.IO;
using System.Text;
using UnityEngine;

namespace AirSimUnity
{
    public static class WebSocketManager
    {

        private static string startWsCall = "api/WebSocket/start";
        private static string endWsCall = "api/WebSocket/end";

        
        public static void StartWS()
        {

            var port = AirSimSettings.GetFlaskPort();
            HttpClient client = new HttpClient();
            client.Get(new System.Uri("http://localhost:" + port + "/api/WebSocket/start"), HttpCompletionOption.AllResponseContent, (r) =>
             {
                 if (r.IsSuccessStatusCode)
                 {
                     Console.WriteLine("Success to start WebSocket");
                 }
                 else
                 {
                     Console.WriteLine("Falied to start WebSocket");
                 }
                 byte[] responseData = r.ReadAsByteArray();

             });

        }




        public static void EndWS()
        {
            var port = AirSimSettings.GetFlaskPort();
            HttpClient client = new HttpClient();
            client.Get(new System.Uri("http://localhost:" + port + "/api/WebSocket/end"), HttpCompletionOption.AllResponseContent, (r) =>
                {
                    if (r.IsSuccessStatusCode)
                    {
                        Console.WriteLine("Success to end WebSocket");
                    }
                    else
                    {
                        Console.WriteLine("Falied to end WebSocket");
                    }
                    byte[] responseData = r.ReadAsByteArray();

                });
        }

    }
}