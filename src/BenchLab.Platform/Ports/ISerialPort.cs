using System;

namespace BenchLab.Platform.Ports;

public interface ISerialPort : IDisposable
{
    string PortName { get; }
    int BytesToRead { get; }
    void Write(byte[] buffer, int offset, int count);
    int Read(byte[] buffer, int offset, int count);
}
