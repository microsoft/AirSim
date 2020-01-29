using System;

namespace CI.HttpClient.Core
{
    public interface IDispatcher
    {
        void Enqueue(Action action);
    }
}