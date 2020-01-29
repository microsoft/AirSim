
namespace CI.HttpClient.Helpers
{
    public static class AuthHelper
    {
        /// <summary>
        /// Create a basic auth header from the supplied username and password
        /// </summary>
        /// <param name="username">A username</param>
        /// <param name="password">A password</param>
        /// <returns>A basic auth header</returns>
        public static string CreateBasicAuthHeader(string username, string password)
        {
            return "Basic " + System.Convert.ToBase64String(System.Text.Encoding.GetEncoding("ISO-8859-1").GetBytes(username + ":" + password));
        }

        /// <summary>
        /// Creates an OAuth 2 header from the supplied OAuth 2 token
        /// </summary>
        /// <param name="token">An OAuth 2 token</param>
        /// <returns>An OAuth 2 header</returns>
        public static string CreateOAuth2Header(string token)
        {
            return "Bearer " + token;
        }
    }
}