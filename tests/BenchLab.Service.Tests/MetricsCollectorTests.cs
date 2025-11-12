using Xunit;
using FluentAssertions;
using BenchLab.Service.Metrics;
using System.Text.RegularExpressions;

namespace BenchLab.Service.Tests;

/// <summary>
/// Tests for lock-free concurrent metrics collection.
/// Validates thread-safety, accuracy, and Prometheus format compliance.
/// </summary>
public class MetricsCollectorTests
{
    [Fact]
    public void RecordRequest_SingleThread_IncrementsCounters()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordRequest("/test", 200, TimeSpan.FromMilliseconds(100));

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_requests_total 1");
        metrics.Should().Contain("benchlab_requests_by_endpoint_total{endpoint=\"/test\"} 1");
        metrics.Should().Contain("benchlab_requests_by_status_total{status=\"200\"} 1");
    }

    [Fact]
    public void RecordRequest_MultipleEndpoints_TracksSeparately()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordRequest("/api/v1", 200, TimeSpan.FromMilliseconds(50));
        collector.RecordRequest("/api/v2", 200, TimeSpan.FromMilliseconds(75));
        collector.RecordRequest("/api/v1", 404, TimeSpan.FromMilliseconds(25));

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_requests_total 3");
        metrics.Should().Contain("benchlab_requests_by_endpoint_total{endpoint=\"/api/v1\"} 2");
        metrics.Should().Contain("benchlab_requests_by_endpoint_total{endpoint=\"/api/v2\"} 1");
        metrics.Should().Contain("benchlab_requests_by_status_total{status=\"200\"} 2");
        metrics.Should().Contain("benchlab_requests_by_status_total{status=\"404\"} 1");
    }

    [Fact]
    public void RecordRequest_TracksRequestDuration()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordRequest("/api", 200, TimeSpan.FromMilliseconds(150));

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_request_duration_seconds_sum{endpoint=\"/api\"}");
        metrics.Should().Contain("benchlab_request_duration_seconds_count{endpoint=\"/api\"} 1");
    }

    [Fact]
    public void RecordRequest_HistogramBuckets_CorrectlyDistribute()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act - Record requests with different durations
        collector.RecordRequest("/api", 200, TimeSpan.FromMilliseconds(50));   // < 0.1s
        collector.RecordRequest("/api", 200, TimeSpan.FromMilliseconds(200));  // < 0.5s
        collector.RecordRequest("/api", 200, TimeSpan.FromMilliseconds(750));  // < 1.0s
        collector.RecordRequest("/api", 200, TimeSpan.FromMilliseconds(1500)); // > 1.0s

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_request_duration_seconds_bucket{endpoint=\"/api\",le=\"0.1\"} 1");
        metrics.Should().Contain("benchlab_request_duration_seconds_bucket{endpoint=\"/api\",le=\"0.5\"} 2");
        metrics.Should().Contain("benchlab_request_duration_seconds_bucket{endpoint=\"/api\",le=\"1.0\"} 3");
        metrics.Should().Contain("benchlab_request_duration_seconds_bucket{endpoint=\"/api\",le=\"+Inf\"} 4");
    }

    [Fact]
    public void RecordDeviceDiscovery_UpdatesDeviceCounts()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordDeviceDiscovery(total: 5, online: 3);

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_devices_discovered_total 5");
        metrics.Should().Contain("benchlab_devices_online 3");
        metrics.Should().Contain("benchlab_devices_offline 2");
    }

    [Fact]
    public void RecordProtocolCommand_IncrementsTotalAndByType()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordProtocolCommand("ReadSensors", success: true);
        collector.RecordProtocolCommand("WriteName", success: true);
        collector.RecordProtocolCommand("ReadSensors", success: false);

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_protocol_commands_total 3");
        metrics.Should().Contain("benchlab_protocol_commands_by_type_total{command=\"ReadSensors\"} 2");
        metrics.Should().Contain("benchlab_protocol_commands_by_type_total{command=\"WriteName\"} 1");
        metrics.Should().Contain("benchlab_protocol_errors_total 1");
    }

    [Fact]
    public void RecordStreamStartEnd_TracksActiveStreams()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordStreamStart();
        collector.RecordStreamStart();
        var metrics1 = collector.ExportPrometheusFormat();

        collector.RecordStreamEnd(bytesTransmitted: 1024, hadError: false);
        var metrics2 = collector.ExportPrometheusFormat();

        // Assert
        metrics1.Should().Contain("benchlab_active_streams 2");
        metrics2.Should().Contain("benchlab_active_streams 1");
        metrics2.Should().Contain("benchlab_bytes_transmitted_total 1024");
        metrics2.Should().Contain("benchlab_stream_errors_total 0");
    }

    [Fact]
    public void RecordStreamEnd_WithError_IncrementsErrorCount()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordStreamStart();
        collector.RecordStreamEnd(bytesTransmitted: 512, hadError: true);

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_stream_errors_total 1");
        metrics.Should().Contain("benchlab_bytes_transmitted_total 512");
    }

    [Fact]
    public void RecordDeviceTelemetry_StoresLatestValues()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordDeviceTelemetry(
            device: "device0",
            chipTemp: 45.5,
            ambientTemp: 25.0,
            power: 150.5,
            fanRpms: new[] { 1200, 1500, 0 }
        );

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_temperature_celsius{sensor=\"device0_chip\"} 45.5");
        metrics.Should().Contain("benchlab_temperature_celsius{sensor=\"device0_ambient\"} 25.0");
        metrics.Should().Contain("benchlab_power_watts{device=\"device0\"} 150.50");
        metrics.Should().Contain("benchlab_fan_rpm{fan=\"device0_fan0\"} 1200");
        metrics.Should().Contain("benchlab_fan_rpm{fan=\"device0_fan1\"} 1500");
        metrics.Should().Contain("benchlab_fan_rpm{fan=\"device0_fan2\"} 0");
    }

    [Fact]
    public void RecordDeviceTelemetry_WithNullValues_SkipsThoseFields()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordDeviceTelemetry(
            device: "device0",
            chipTemp: null,
            ambientTemp: 25.0,
            power: null,
            fanRpms: null
        );

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().NotContain("device0_chip");
        metrics.Should().Contain("benchlab_temperature_celsius{sensor=\"device0_ambient\"} 25.0");
        metrics.Should().NotContain("benchlab_power_watts{device=\"device0\"}");
        metrics.Should().NotContain("benchlab_fan_rpm{fan=\"device0_fan0\"}");
    }

    [Fact]
    public void RecordDeviceTelemetry_UpdatesLastSuccessfulRead()
    {
        // Arrange
        var collector = new MetricsCollector();
        var beforeTimestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds();

        // Act
        collector.RecordDeviceTelemetry(
            device: "device0",
            chipTemp: 45.0,
            ambientTemp: null,
            power: null,
            fanRpms: null
        );

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        var afterTimestamp = DateTimeOffset.UtcNow.ToUnixTimeSeconds();

        // Extract timestamp from metrics
        var timestampMatch = Regex.Match(metrics, @"benchlab_last_successful_read_timestamp_seconds\{device=""device0""\} (\d+)");
        timestampMatch.Success.Should().BeTrue();
        var recordedTimestamp = long.Parse(timestampMatch.Groups[1].Value);

        recordedTimestamp.Should().BeInRange(beforeTimestamp, afterTimestamp);
    }

    [Fact]
    public void ExportPrometheusFormat_IncludesServiceUptime()
    {
        // Arrange
        var collector = new MetricsCollector();
        Thread.Sleep(100); // Let some time pass

        // Act
        var metrics = collector.ExportPrometheusFormat();

        // Assert
        metrics.Should().Contain("# HELP benchlab_uptime_seconds");
        metrics.Should().Contain("# TYPE benchlab_uptime_seconds counter");
        metrics.Should().MatchRegex(@"benchlab_uptime_seconds \d+\.\d+");
    }

    [Fact]
    public void ExportPrometheusFormat_IncludesHelpAndTypeComments()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        var metrics = collector.ExportPrometheusFormat();

        // Assert
        metrics.Should().Contain("# HELP benchlab_requests_total");
        metrics.Should().Contain("# TYPE benchlab_requests_total counter");
        metrics.Should().Contain("# HELP benchlab_devices_discovered_total");
        metrics.Should().Contain("# TYPE benchlab_devices_discovered_total gauge");
    }

    [Fact]
    public void ExportPrometheusFormat_ValidFormat()
    {
        // Arrange
        var collector = new MetricsCollector();
        collector.RecordRequest("/test", 200, TimeSpan.FromMilliseconds(50));

        // Act
        var metrics = collector.ExportPrometheusFormat();

        // Assert
        // Should follow Prometheus text format
        var lines = metrics.Split('\n', StringSplitOptions.RemoveEmptyEntries);
        lines.Should().Contain(line => line.StartsWith("# HELP"));
        lines.Should().Contain(line => line.StartsWith("# TYPE"));
        lines.Should().Contain(line => Regex.IsMatch(line, @"^[a-z_][a-z0-9_]+ \d+"));
    }

    [Fact]
    public async Task ConcurrentRecordRequest_ThreadSafe()
    {
        // Arrange
        var collector = new MetricsCollector();
        const int threadCount = 10;
        const int requestsPerThread = 100;

        // Act - Record requests concurrently from multiple threads
        var tasks = Enumerable.Range(0, threadCount)
            .Select(i => Task.Run(() =>
            {
                for (int j = 0; j < requestsPerThread; j++)
                {
                    collector.RecordRequest("/test", 200, TimeSpan.FromMilliseconds(10));
                }
            }))
            .ToArray();

        await Task.WhenAll(tasks);

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        var expectedTotal = threadCount * requestsPerThread;
        metrics.Should().Contain($"benchlab_requests_total {expectedTotal}");
    }

    [Fact]
    public async Task ConcurrentProtocolCommands_ThreadSafe()
    {
        // Arrange
        var collector = new MetricsCollector();
        const int threadCount = 10;
        const int commandsPerThread = 100;

        // Act
        var tasks = Enumerable.Range(0, threadCount)
            .Select(i => Task.Run(() =>
            {
                for (int j = 0; j < commandsPerThread; j++)
                {
                    collector.RecordProtocolCommand("TestCommand", success: j % 10 != 0);
                }
            }))
            .ToArray();

        await Task.WhenAll(tasks);

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        var expectedTotal = threadCount * commandsPerThread;
        var expectedErrors = threadCount * 10; // Every 10th fails

        metrics.Should().Contain($"benchlab_protocol_commands_total {expectedTotal}");
        metrics.Should().Contain($"benchlab_protocol_errors_total {expectedErrors}");
    }

    [Fact]
    public async Task ConcurrentStreamOperations_ThreadSafe()
    {
        // Arrange
        var collector = new MetricsCollector();
        const int streamCount = 20;

        // Act - Start and end streams concurrently
        var startTasks = Enumerable.Range(0, streamCount)
            .Select(_ => Task.Run(() => collector.RecordStreamStart()))
            .ToArray();

        await Task.WhenAll(startTasks);

        var endTasks = Enumerable.Range(0, streamCount)
            .Select(i => Task.Run(() => collector.RecordStreamEnd(1024, hadError: i % 5 == 0)))
            .ToArray();

        await Task.WhenAll(endTasks);

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain("benchlab_active_streams 0"); // All ended
        metrics.Should().Contain($"benchlab_bytes_transmitted_total {streamCount * 1024}");
        metrics.Should().Contain("benchlab_stream_errors_total 4"); // Every 5th
    }

    [Fact]
    public async Task ConcurrentTelemetryUpdates_ThreadSafe()
    {
        // Arrange
        var collector = new MetricsCollector();
        const int updateCount = 100;

        // Act - Update telemetry concurrently
        var tasks = Enumerable.Range(0, updateCount)
            .Select(i => Task.Run(() =>
            {
                collector.RecordDeviceTelemetry(
                    device: $"device{i % 5}",
                    chipTemp: 40.0 + i,
                    ambientTemp: 25.0,
                    power: 100.0 + i,
                    fanRpms: new[] { 1000 + i, 1200 + i }
                );
            }))
            .ToArray();

        await Task.WhenAll(tasks);

        // Assert
        var metrics = collector.ExportPrometheusFormat();

        // Should have latest values for each of the 5 devices
        for (int i = 0; i < 5; i++)
        {
            metrics.Should().Contain($"device{i}_chip");
            metrics.Should().Contain($"device=\"device{i}\"");
        }
    }

    [Fact]
    public async Task ConcurrentExport_DuringUpdates_NoException()
    {
        // Arrange
        var collector = new MetricsCollector();
        var cts = new CancellationTokenSource();

        // Act - Continuously update metrics while exporting
        var updateTask = Task.Run(async () =>
        {
            int counter = 0;
            while (!cts.Token.IsCancellationRequested)
            {
                collector.RecordRequest("/test", 200, TimeSpan.FromMilliseconds(10));
                collector.RecordProtocolCommand("TestCommand", success: true);
                counter++;
                if (counter > 1000) break;
                await Task.Delay(1);
            }
        });

        var exportTasks = Enumerable.Range(0, 10)
            .Select(_ => Task.Run(() =>
            {
                for (int i = 0; i < 100; i++)
                {
                    var metrics = collector.ExportPrometheusFormat();
                    metrics.Should().NotBeNullOrEmpty();
                }
            }))
            .ToArray();

        await Task.WhenAll(exportTasks);
        cts.Cancel();
        await updateTask;

        // Assert - No exceptions should be thrown
        // If we get here, the test passed
    }

    [Fact]
    public void EmptyCollector_ExportsValidMetrics()
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        var metrics = collector.ExportPrometheusFormat();

        // Assert
        metrics.Should().NotBeNullOrEmpty();
        metrics.Should().Contain("benchlab_uptime_seconds");
        metrics.Should().Contain("benchlab_requests_total 0");
    }

    [Theory]
    [InlineData("/", 200)]
    [InlineData("/api/v1/users", 404)]
    [InlineData("/admin/settings", 403)]
    [InlineData("/health", 200)]
    public void RecordRequest_VariousEndpointsAndStatuses_TrackedCorrectly(string endpoint, int status)
    {
        // Arrange
        var collector = new MetricsCollector();

        // Act
        collector.RecordRequest(endpoint, status, TimeSpan.FromMilliseconds(100));

        // Assert
        var metrics = collector.ExportPrometheusFormat();
        metrics.Should().Contain($"benchlab_requests_by_endpoint_total{{endpoint=\"{endpoint}\"}} 1");
        metrics.Should().Contain($"benchlab_requests_by_status_total{{status=\"{status}\"}} 1");
    }
}
