using System;
using System.IO.Ports;
using System.Text;
using System.Threading;
using BenchLab.Platform.Ports;

namespace BenchLab.Platform.Discovery;

/// <summary>
/// Simple textual/hex handshake that works for many USB-CDC devices.
/// Strategy:
///  - Open port at 115200 8N1, DTR/RTS true
///  - Optionally write a ping token (configurable)
///  - Read any available bytes within timeout
///  - Match using a predicate (contains token or any bytes at all)
/// </summary>
public sealed class SimpleHandshake : IBenchlabHandshake
{
    private readonly string? _ping;
    private readonly Func<byte[], bool> _matcher;

    public SimpleHandshake(string? ping = null, Func<byte[], bool>? matcher = null)
    {
        _ping = ping;
        _matcher = matcher ?? (buf => buf.Length > 0);
    }

    public ProbeResult Probe(string device, TimeSpan timeout)
    {
        using var sp = SerialPortAdapter.Open(device, 115200, timeout, timeout, dtr:true, rts:true);
        var info = "opened";

        if (!string.IsNullOrEmpty(_ping))
        {
            var bytes = Encoding.ASCII.GetBytes(_ping!);
            sp.Write(bytes, 0, bytes.Length);
            info += " pinged";
        }

        // small wait for device to respond
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var buffer = new byte[256];
        int total = 0;
        while (sw.Elapsed < timeout && total < buffer.Length)
        {
            var avail = sp.BytesToRead;
            if (avail > 0)
            {
                var n = sp.Read(buffer, total, Math.Min(avail, buffer.Length - total));
                total += n;
                // heuristic: if we already have something, we can stop early
                if (total > 0) break;
            }
            Thread.Sleep(10);
        }

        var slice = new byte[total];
        Array.Copy(buffer, slice, total);

        var ok = _matcher(slice);
        var hex = BitConverter.ToString(slice).Replace("-", " ");
        info += $" bytes={total} hex=[{hex}]";
        return ok ? ProbeResult.Ok(device, info) : ProbeResult.Failed(device, info);
    }
}
