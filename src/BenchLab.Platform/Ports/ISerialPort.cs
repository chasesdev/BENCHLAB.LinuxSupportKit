using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace BenchLab.Platform.Ports;

public interface ISerialPort : IDisposable
{
    string PortName { get; }
    int BytesToRead { get; }
    Stream BaseStream { get; }

    void Write(byte[] buffer, int offset, int count);
    int Read(byte[] buffer, int offset, int count);

    Task WriteAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken = default);
    Task<int> ReadAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken = default);

    void DiscardInBuffer();
}
