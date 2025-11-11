using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Microsoft.Extensions.Logging;
using BenchLab.Platform.Ports;

namespace BenchLab.Platform.Discovery;

/// <summary>
/// High-level discovery: prefers /dev/serial/by-id, falls back to /dev/ttyACM*,
/// and verifies candidates via a pluggable probe/handshake.
/// </summary>
public sealed class PortDiscovery
{
    private readonly ILogger? _log;
    private readonly IBenchlabHandshake _handshake;

    public PortDiscovery(IBenchlabHandshake handshake, ILogger? log = null)
    {
        _handshake = handshake;
        _log = log;
    }

    public IEnumerable<ProbeResult> Discover(TimeSpan? perPortTimeout = null)
    {
        var timeout = perPortTimeout ?? TimeSpan.FromMilliseconds(500);
        var candidates = new List<string>();

        var byId = "/dev/serial/by-id";
        if (Directory.Exists(byId))
        {
            foreach (var link in Directory.EnumerateFiles(byId))
            {
                try
                {
                    // resolve symlink to device path if possible
                    var full = Path.GetFullPath(link);
                    candidates.Add(full);
                }
                catch (Exception ex)
                {
                    _log?.LogWarning(ex, "Failed to resolve symlink {Link}", link);
                }
            }
        }

        // Fallback to ttyACM*
        try
        {
            foreach (var dev in Directory.EnumerateFiles("/dev", "ttyACM*"))
                candidates.Add(dev);
        }
        catch (Exception ex)
        {
            _log?.LogWarning(ex, "Failed to enumerate /dev/ttyACM* devices");
        }

        foreach (var dev in candidates.Distinct())
        {
            ProbeResult result;
            try
            {
                result = _handshake.Probe(dev, timeout);
            }
            catch (Exception ex)
            {
                result = ProbeResult.Failed(dev, $"probe-error: {ex.GetType().Name}: {ex.Message}");
            }
            _log?.LogInformation("Probe {Device} => {Ok} {Info}", dev, result.IsBenchlab, result.Info);
            yield return result;
        }
    }
}

public sealed record ProbeResult(string Device, bool IsBenchlab, string Info)
{
    public static ProbeResult Ok(string dev, string info) => new(dev, true, info);
    public static ProbeResult Failed(string dev, string info) => new(dev, false, info);
}

public interface IBenchlabHandshake
{
    ProbeResult Probe(string device, TimeSpan timeout);
}
