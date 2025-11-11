using System.Diagnostics;
using System.Text;

namespace BenchLab.Service.Metrics;

/// <summary>
/// Thread-safe metrics collector for Prometheus-compatible metrics export.
/// Tracks HTTP requests, device operations, protocol commands, and device telemetry.
/// </summary>
public sealed class MetricsCollector
{
    private readonly object _lock = new();
    private readonly Stopwatch _uptimeStopwatch = Stopwatch.StartNew();

    // HTTP Metrics
    private long _totalRequests;
    private readonly Dictionary<string, long> _requestsByEndpoint = new();
    private readonly Dictionary<string, long> _requestsByStatus = new();
    private readonly Dictionary<string, List<double>> _requestDurations = new();

    // Device Metrics
    private long _devicesDiscovered;
    private long _devicesOnline;
    private long _devicesOffline;

    // Protocol Metrics
    private long _totalProtocolCommands;
    private readonly Dictionary<string, long> _commandsByType = new();
    private long _protocolErrors;

    // Stream Metrics
    private long _activeStreams;
    private long _totalBytesTransmitted;
    private long _streamErrors;

    // Device Telemetry (latest values)
    private readonly Dictionary<string, double> _latestTemperatures = new();
    private readonly Dictionary<string, double> _latestPowerReadings = new();
    private readonly Dictionary<string, int> _latestFanSpeeds = new();
    private readonly Dictionary<string, DateTime> _lastSuccessfulRead = new();

    public void RecordRequest(string endpoint, int statusCode, TimeSpan duration)
    {
        lock (_lock)
        {
            _totalRequests++;
            _requestsByEndpoint.TryGetValue(endpoint, out var count);
            _requestsByEndpoint[endpoint] = count + 1;

            var statusKey = statusCode.ToString();
            _requestsByStatus.TryGetValue(statusKey, out var statusCount);
            _requestsByStatus[statusKey] = statusCount + 1;

            if (!_requestDurations.ContainsKey(endpoint))
                _requestDurations[endpoint] = new List<double>();
            _requestDurations[endpoint].Add(duration.TotalSeconds);
        }
    }

    public void RecordDeviceDiscovery(int total, int online)
    {
        lock (_lock)
        {
            _devicesDiscovered = total;
            _devicesOnline = online;
            _devicesOffline = total - online;
        }
    }

    public void RecordProtocolCommand(string commandName, bool success)
    {
        lock (_lock)
        {
            _totalProtocolCommands++;
            _commandsByType.TryGetValue(commandName, out var count);
            _commandsByType[commandName] = count + 1;

            if (!success)
                _protocolErrors++;
        }
    }

    public void RecordStreamStart()
    {
        lock (_lock)
        {
            _activeStreams++;
        }
    }

    public void RecordStreamEnd(long bytesTransmitted, bool hadError)
    {
        lock (_lock)
        {
            _activeStreams--;
            _totalBytesTransmitted += bytesTransmitted;
            if (hadError)
                _streamErrors++;
        }
    }

    public void RecordDeviceTelemetry(string device, double? chipTemp, double? ambientTemp,
        double? power, int[]? fanRpms)
    {
        lock (_lock)
        {
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
    }

    public string ExportPrometheusFormat()
    {
        lock (_lock)
        {
            var sb = new StringBuilder();

            // Service uptime
            sb.AppendLine("# HELP benchlab_uptime_seconds Service uptime in seconds");
            sb.AppendLine("# TYPE benchlab_uptime_seconds counter");
            sb.AppendLine($"benchlab_uptime_seconds {_uptimeStopwatch.Elapsed.TotalSeconds:F2}");
            sb.AppendLine();

            // Total requests
            sb.AppendLine("# HELP benchlab_requests_total Total HTTP requests");
            sb.AppendLine("# TYPE benchlab_requests_total counter");
            sb.AppendLine($"benchlab_requests_total {_totalRequests}");
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
                var durations = kvp.Value.OrderBy(d => d).ToList();
                if (durations.Count > 0)
                {
                    var p50 = Percentile(durations, 0.5);
                    var p90 = Percentile(durations, 0.9);
                    var p99 = Percentile(durations, 0.99);
                    var sum = durations.Sum();
                    var count = durations.Count;

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
