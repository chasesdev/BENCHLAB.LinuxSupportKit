using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using BenchLab.Platform.Ports;

namespace BenchLab.Platform.Discovery;

/// <summary>
/// High-level discovery: prefers /dev/serial/by-id, falls back to /dev/ttyACM*,
/// and verifies candidates via a pluggable probe/handshake.
/// Uses parallel probing with caching for optimal performance.
/// </summary>
public sealed class PortDiscovery
{
    private readonly ILogger? _log;
    private readonly IBenchlabHandshake _handshake;
    private readonly ConcurrentDictionary<string, (ProbeResult result, DateTime cachedAt)> _cache = new();
    private readonly TimeSpan _cacheTtl = TimeSpan.FromSeconds(60);
    private readonly SemaphoreSlim _probeSemaphore = new(5); // Max 5 concurrent probes

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

        // Parallel probing with caching
        var distinctCandidates = candidates.Distinct().ToList();
        var probeTasks = distinctCandidates.Select(dev => ProbeDeviceAsync(dev, timeout)).ToArray();

        try
        {
            Task.WaitAll(probeTasks);
        }
        catch
        {
            // Individual probe errors are already handled
        }

        return probeTasks.Select(t => t.Result).ToList();
    }

    private async Task<ProbeResult> ProbeDeviceAsync(string device, TimeSpan timeout)
    {
        // Check cache first
        if (_cache.TryGetValue(device, out var cached))
        {
            if (DateTime.UtcNow - cached.cachedAt < _cacheTtl)
            {
                _log?.LogDebug("Cache hit for {Device}", device);
                return cached.result;
            }
            else
            {
                // Expired, remove from cache
                _cache.TryRemove(device, out _);
            }
        }

        // Throttle concurrent probes to avoid overwhelming system
        await _probeSemaphore.WaitAsync();
        try
        {
            ProbeResult result;
            try
            {
                // Probe is synchronous, run on thread pool to avoid blocking
                result = await Task.Run(() => _handshake.Probe(device, timeout));
            }
            catch (Exception ex)
            {
                result = ProbeResult.Failed(device, $"probe-error: {ex.GetType().Name}: {ex.Message}");
            }

            _log?.LogInformation("Probe {Device} => {Ok} {Info}", device, result.IsBenchlab, result.Info);

            // Cache the result
            _cache[device] = (result, DateTime.UtcNow);

            return result;
        }
        finally
        {
            _probeSemaphore.Release();
        }
    }

    /// <summary>
    /// Clear discovery cache, forcing re-probe on next Discover() call.
    /// </summary>
    public void ClearCache()
    {
        _cache.Clear();
        _log?.LogInformation("Discovery cache cleared");
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
