using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;

namespace BenchLab.Platform.Ports;

public sealed class SerialPortAdapter : ISerialPort
{
    private readonly SerialPort _sp;

    private SerialPortAdapter(SerialPort sp)
    {
        _sp = sp;
    }

    public static SerialPortAdapter Open(string port, int baud, TimeSpan readTimeout, TimeSpan writeTimeout, bool dtr, bool rts)
    {
        var sp = new SerialPort(port, baud, Parity.None, 8, StopBits.One)
        {
            ReadTimeout = (int)readTimeout.TotalMilliseconds,
            WriteTimeout = (int)writeTimeout.TotalMilliseconds,
            DtrEnable = dtr,
            RtsEnable = rts,
            NewLine = "\n",
            Handshake = Handshake.None,
            // Optimize buffer sizes for BenchLab protocol
            // Protocol uses ~194 byte packets, so 512 bytes provides headroom
            ReadBufferSize = 512,   // Smaller than default 4096, matches protocol packet sizes
            WriteBufferSize = 128   // Commands are small (1-33 bytes typically)
        };
        sp.Open();
        return new SerialPortAdapter(sp);
    }

    public string PortName => _sp.PortName;
    public int BytesToRead => _sp.BytesToRead;
    public Stream BaseStream => _sp.BaseStream;

    public void Write(byte[] buffer, int offset, int count) => _sp.Write(buffer, offset, count);
    public int Read(byte[] buffer, int offset, int count) => _sp.Read(buffer, offset, count);

    public async Task WriteAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken = default)
    {
        await _sp.BaseStream.WriteAsync(buffer, offset, count, cancellationToken);
    }

    public async Task<int> ReadAsync(byte[] buffer, int offset, int count, CancellationToken cancellationToken = default)
    {
        return await _sp.BaseStream.ReadAsync(buffer, offset, count, cancellationToken);
    }

    public void DiscardInBuffer()
    {
        _sp.DiscardInBuffer();
    }

    public void Dispose()
    {
        try { _sp.Dispose(); } catch { /* ignore */ }
    }
}
