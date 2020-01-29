
using CI.HttpClient;
using System;
namespace AirSimUnity
{
    public static class WebSocketManager
    {

        private static Uri WSUri = new Uri("http://localhost:5000/");
        private static string startWsCall = "api/WebSocket/start";
        private static string endWsCall = "api/WebSocket/end";

        public static  void StartWS()
        {

            HttpClient client = new HttpClient();
            client.Get(new System.Uri("http://localhost:5000/api/WebSocket/start"), HttpCompletionOption.AllResponseContent, (r) =>
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


            //using (var client = new HttpClient())
            //{
            //    client.BaseAddress = WSUri;
            //    client.DefaultRequestHeaders.Accept.Clear();
            //    client.DefaultRequestHeaders.Accept.Add(new MediaTypeWithQualityHeaderValue("application/json"));

            //    // HTTP GET
            //    HttpResponseMessage response = await client.GetAsync(startWsCall);
            //    if (response.IsSuccessStatusCode)
            //    {
            //        Console.WriteLine("Success to start WebSocket");
            //    }
            //    else
            //    {
            //        Console.WriteLine("Falied to start WebSocket");
            //    }
            //}
        }




        public static  void EndWS()
        {

            HttpClient client = new HttpClient();
            client.Get(new System.Uri("http://localhost:5000/api/WebSocket/end"), HttpCompletionOption.AllResponseContent, (r) =>
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

            //using (var client = new HttpClient())
            //{
            //    client.BaseAddress = WSUri;
            //    client.DefaultRequestHeaders.Accept.Clear();
            //    client.DefaultRequestHeaders.Accept.Add(new MediaTypeWithQualityHeaderValue("application/json"));

            //    // HTTP GET
            //    HttpResponseMessage response = await client.GetAsync(endWsCall);
            //    if (response.IsSuccessStatusCode)
            //    {
            //        Console.WriteLine("Success to end WebSocket");
            //    }
            //    else
            //    {
            //        Console.WriteLine("Falied to end WebSocket");
            //    }
            //}
        }

    }
}