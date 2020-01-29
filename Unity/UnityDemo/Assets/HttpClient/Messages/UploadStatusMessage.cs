
namespace CI.HttpClient
{
    /// <summary>
    /// Represents the status of an upload
    /// </summary>
    public class UploadStatusMessage
    {
        /// <summary>
        /// Length of the content being uploaded
        /// </summary>
        public long ContentLength
        {
            get; set;
        }

        /// <summary>
        /// How much content as been uploaded so far
        /// </summary>
        public long TotalContentUploaded
        {
            get; set;
        }

        /// <summary>
        /// How much content has been uploaded since the last upload status message was raised
        /// </summary>
        public long ContentUploadedThisRound
        {
            get; set;
        }

        /// <summary>
        /// Percentage completion of the upload
        /// </summary>
        public int PercentageComplete
        {
            get { return (int)(((double)TotalContentUploaded / ContentLength) * 100); }
        }
    }
}