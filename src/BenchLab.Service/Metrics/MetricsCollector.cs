using System.Collections.Concurrent;
using System.Diagnostics;
using System.Text;

namespace BenchLab.Service.Metrics;

/// <summary>
/// Lock-free thread-safe metrics collector for Prometheus-compatible metrics export.
/// Uses concurrent collections and Interlocked operations for high-performance concurrent access.
/// Tracks HTTP requests, device operations, protocol commands, and device telemetry.
/// </summary>
public sealed class MetricsCollector
{
    private readonly Stopwatch _uptimeStopwatch = Stopwatch.StartNew();

    // HTTP Metrics - Lock-free counters
    private long _totalRequests;
    private readonly ConcurrentDictionary<string, long> _requestsByEndpoint = new();
    private readonly ConcurrentDictionary<string, long> _requestsByStatus = new();
    private readonly ConcurrentDictionary<string, ConcurrentBag<double>> _requestDurations = new();

    // Device Metrics - Interlocked updates
    private long _devicesDiscovered;
    private long _devicesOnline;
    private long _devicesOffline;

    // Protocol Metrics - Lock-free
    private long _totalProtocolCommands;
    private readonly ConcurrentDictionary<string, long> _commandsByType = new();
    private long _protocolErrors;

    // Stream Metrics - Interlocked operations
    private long _activeStreams;
    private long _totalBytesTransmitted;
    private long _streamErrors;

    // Device Telemetry (latest values) - Concurrent dictionaries
    private readonly ConcurrentDictionary<string, double> _latestTemperatures = new();
    private readonly ConcurrentDictionary<string, double> _latestPowerReadings = new();
    private readonly ConcurrentDictionary<string, int> _latestFanSpeeds = new();
    private readonly ConcurrentDictionary<string, DateTime> _lastSuccessfulRead = new();

    public void RecordRequest(string endpoint, int statusCode, TimeSpan duration)
    {
        // Lock-free using Interlocked and ConcurrentDictionary
        Interlocked.Increment(ref _totalRequests);
        _requestsByEndpoint.AddOrUpdate(endpoint, 1, (_, count) => count + 1);

        var statusKey = statusCode.ToString();
        _requestsByStatus.AddOrUpdate(statusKey, 1, (_, count) => count + 1);

        var durations = _requestDurations.GetOrAdd(endpoint, _ => new ConcurrentBag<double>());
        durations.Add(duration.TotalSeconds);
    }

    public void RecordDeviceDiscovery(int total, int online)
    {
        // Lock-free using Interlocked.Exchange
        Interlocked.Exchange(ref _devicesDiscovered, total);
        Interlocked.Exchange(ref _devicesOnline, online);
        Interlocked.Exchange(ref _devicesOffline, total - online);
    }

    public void RecordProtocolCommand(string commandName, bool success)
    {
        // Lock-free using Interlocked and ConcurrentDictionary
        Interlocked.Increment(ref _totalProtocolCommands);
        _commandsByType.AddOrUpdate(commandName, 1, (_, count) => count + 1);

        if (!success)
            Interlocked.Increment(ref _protocolErrors);
    }

    public void RecordStreamStart()
    {
        Interlocked.Increment(ref _activeStreams);
    }

    public void RecordStreamEnd(long bytesTransmitted, bool hadError)
    {
        Interlocked.Decrement(ref _activeStreams);
        Interlocked.Add(ref _totalBytesTransmitted, bytesTransmitted);
        if (hadError)
            Interlocked.Increment(ref _streamErrors);
    }

    public void RecordDeviceTelemetry(string device, double? chipTemp, double? ambientTemp,
        double? power, int[]? fanRpms)
    {
        // Lock-free using ConcurrentDictionary
        if (chipTemp.HasValue)
            _latestTemperatures[$"{device}_chip"] = chipTemp.Value;

        if (ambientTemp.HasValue)
            _latestTemperatures[$"{device}_ambient"] = ambientTemp.Value;

        if (power.HasValue)
            _latestPowerReadings[device] = power.Value;

        if (fanRpms != null)
        {
            for (int i = 0; i < fanRpms.Length; i++)
                _latestFanSpeeds[$"{device}_fan{i}"] = fanRpms[i];
        }

        _lastSuccessfulRead[device] = DateTime.UtcNow;
    }

    public string ExportPrometheusFormat()
    {
        // Lock-free export using concurrent collections and Interlocked reads
        var sb = new StringBuilder();

        // Service uptime
        sb.AppendLine("# HELP benchlab_uptime_seconds Service uptime in seconds");
        sb.AppendLine("# TYPE benchlab_uptime_seconds counter");
        sb.AppendLine($"benchlab_uptime_seconds {_uptimeStopwatch.Elapsed.TotalSeconds:F2}");
        sb.AppendLine();

        // Total requests (snapshot with Interlocked.Read)
        sb.AppendLine("# HELP benchlab_requests_total Total HTTP requests");
        sb.AppendLine("# TYPE benchlab_requests_total counter");
        sb.AppendLine($"benchlab_requests_total {Interlocked.Read(ref _totalRequests)}");
        sb.AppendLine();

            // Requests by endpoint
            sb.AppendLine("# HELP benchlab_requests_by_endpoint_total Total HTTP requests by endpoint");
            sb.AppendLine("# TYPE benchlab_requests_by_endpoint_total counter");
            foreach (var kvp in _requestsByEndpoint)
                sb.AppendLine($"benchlab_requests_by_endpoint_total{{endpoint=\"{kvp.Key}\"}} {kvp.Value}");
            sb.AppendLine();

            // Requests by status code
            sb.AppendLine("# HELP benchlab_requests_by_status_total Total HTTP requests by status code");
            sb.AppendLine("# TYPE benchlab_requests_by_status_total counter");
            foreach (var kvp in _requestsByStatus)
                sb.AppendLine($"benchlab_requests_by_status_total{{status=\"{kvp.Key}\"}} {kvp.Value}");
            sb.AppendLine();

            // Request duration histograms
            sb.AppendLine("# HELP benchlab_request_duration_seconds HTTP request duration in seconds");
            sb.AppendLine("# TYPE benchlab_request_duration_seconds histogram");
            foreach (var kvp in _requestDurations)
            {
                // Snapshot ConcurrentBag to array for processing
                var durations = kvp.Value.ToArray();
                if (durations.Length > 0)
                {
                    Array.Sort(durations);
                    var sum = durations.Sum();
                    var count = durations.Length;

                    sb.AppendLine($"benchlab_request_duration_seconds_bucket{{endpoint=\"{kvp.Key}\",le=\"0.1\"}} {durations.Count(d => d <= 0.1)}");
                    sb.AppendLine($"benchlab_request_duration_seconds_bucket{{endpoint=\"{kvp.Key}\",le=\"0.5\"}} {durations.Count(d => d <= 0.5)}");
                    sb.AppendLine($"benchlab_request_duration_seconds_bucket{{endpoint=\"{kvp.Key}\",le=\"1.0\"}} {durations.Count(d => d <= 1.0)}");
                    sb.AppendLine($"benchlab_request_duration_seconds_bucket{{endpoint=\"{kvp.Key}\",le=\"+Inf\"}} {count}");
                    sb.AppendLine($"benchlab_request_duration_seconds_sum{{endpoint=\"{kvp.Key}\"}} {sum:F4}");
                    sb.AppendLine($"benchlab_request_duration_seconds_count{{endpoint=\"{kvp.Key}\"}} {count}");
                }
            }
            sb.AppendLine();

            // Device counts
            sb.AppendLine("# HELP benchlab_devices_discovered_total Total number of discovered devices");
            sb.AppendLine("# TYPE benchlab_devices_discovered_total gauge");
            sb.AppendLine($"benchlab_devices_discovered_total {_devicesDiscovered}");
            sb.AppendLine();

            sb.AppendLine("# HELP benchlab_devices_online Number of online BenchLab devices");
            sb.AppendLine("# TYPE benchlab_devices_online gauge");
            sb.AppendLine($"benchlab_devices_online {_devicesOnline}");
            sb.AppendLine();

            sb.AppendLine("# HELP benchlab_devices_offline Number of offline devices");
            sb.AppendLine("# TYPE benchlab_devices_offline gauge");
            sb.AppendLine($"benchlab_devices_offline {_devicesOffline}");
            sb.AppendLine();

            // Protocol metrics
            sb.AppendLine("# HELP benchlab_protocol_commands_total Total protocol commands executed");
            sb.AppendLine("# TYPE benchlab_protocol_commands_total counter");
            sb.AppendLine($"benchlab_protocol_commands_total {_totalProtocolCommands}");
            sb.AppendLine();

            sb.AppendLine("# HELP benchlab_protocol_commands_by_type_total Protocol commands by type");
            sb.AppendLine("# TYPE benchlab_protocol_commands_by_type_total counter");
            foreach (var kvp in _commandsByType)
                sb.AppendLine($"benchlab_protocol_commands_by_type_total{{command=\"{kvp.Key}\"}} {kvp.Value}");
            sb.AppendLine();

            sb.AppendLine("# HELP benchlab_protocol_errors_total Total protocol errors");
            sb.AppendLine("# TYPE benchlab_protocol_errors_total counter");
            sb.AppendLine($"benchlab_protocol_errors_total {_protocolErrors}");
            sb.AppendLine();

            // Stream metrics
            sb.AppendLine("# HELP benchlab_active_streams Number of active streaming connections");
            sb.AppendLine("# TYPE benchlab_active_streams gauge");
            sb.AppendLine($"benchlab_active_streams {_activeStreams}");
            sb.AppendLine();

            sb.AppendLine("# HELP benchlab_bytes_transmitted_total Total bytes transmitted in streams");
            sb.AppendLine("# TYPE benchlab_bytes_transmitted_total counter");
            sb.AppendLine($"benchlab_bytes_transmitted_total {_totalBytesTransmitted}");
            sb.AppendLine();

            sb.AppendLine("# HELP benchlab_stream_errors_total Total stream errors");
            sb.AppendLine("# TYPE benchlab_stream_errors_total counter");
            sb.AppendLine($"benchlab_stream_errors_total {_streamErrors}");
            sb.AppendLine();

            // Device temperatures
            if (_latestTemperatures.Count > 0)
            {
                sb.AppendLine("# HELP benchlab_temperature_celsius Current device temperature in Celsius");
                sb.AppendLine("# TYPE benchlab_temperature_celsius gauge");
                foreach (var kvp in _latestTemperatures)
                    sb.AppendLine($"benchlab_temperature_celsius{{sensor=\"{kvp.Key}\"}} {kvp.Value:F1}");
                sb.AppendLine();
            }

            // Device power
            if (_latestPowerReadings.Count > 0)
            {
                sb.AppendLine("# HELP benchlab_power_watts Current power consumption in watts");
                sb.AppendLine("# TYPE benchlab_power_watts gauge");
                foreach (var kvp in _latestPowerReadings)
                    sb.AppendLine($"benchlab_power_watts{{device=\"{kvp.Key}\"}} {kvp.Value:F2}");
                sb.AppendLine();
            }

            // Fan speeds
            if (_latestFanSpeeds.Count > 0)
            {
                sb.AppendLine("# HELP benchlab_fan_rpm Current fan speed in RPM");
                sb.AppendLine("# TYPE benchlab_fan_rpm gauge");
                foreach (var kvp in _latestFanSpeeds)
                    sb.AppendLine($"benchlab_fan_rpm{{fan=\"{kvp.Key}\"}} {kvp.Value}");
                sb.AppendLine();
            }

            // Last successful read
            if (_lastSuccessfulRead.Count > 0)
            {
                sb.AppendLine("# HELP benchlab_last_successful_read_timestamp_seconds Unix timestamp of last successful device read");
                sb.AppendLine("# TYPE benchlab_last_successful_read_timestamp_seconds gauge");
                foreach (var kvp in _lastSuccessfulRead)
                {
                    var unixTimestamp = new DateTimeOffset(kvp.Value).ToUnixTimeSeconds();
                    sb.AppendLine($"benchlab_last_successful_read_timestamp_seconds{{device=\"{kvp.Key}\"}} {unixTimestamp}");
                }
                sb.AppendLine();
            }

            return sb.ToString();
        }
    }

    private static double Percentile(List<double> sortedValues, double percentile)
    {
        if (sortedValues.Count == 0) return 0;
        var index = (int)Math.Ceiling(percentile * sortedValues.Count) - 1;
        return sortedValues[Math.Max(0, Math.Min(index, sortedValues.Count - 1))];
    }
}
