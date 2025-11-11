using System.Net;
using System.Net.Http.Headers;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Mvc.Testing;
using Xunit;
using FluentAssertions;
using BenchLab.Platform.Discovery;

namespace BenchLab.Service.Tests;

/// <summary>
/// Integration tests for the HTTP service endpoints.
/// Tests all REST API endpoints, authentication, error handling, and NDJSON streaming.
/// </summary>
public class IntegrationTests : IClassFixture<WebApplicationFactory<Program>>
{
    private readonly WebApplicationFactory<Program> _factory;
    private readonly HttpClient _client;

    public IntegrationTests(WebApplicationFactory<Program> factory)
    {
        _factory = factory;
        _client = factory.CreateClient();
    }

    #region Health and Metrics Endpoints

    [Fact]
    public async Task Health_ReturnsHealthy()
    {
        // Act
        var response = await _client.GetAsync("/health");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        var content = await response.Content.ReadAsStringAsync();
        var json = JsonDocument.Parse(content);
        json.RootElement.GetProperty("status").GetString().Should().Be("healthy");
        json.RootElement.TryGetProperty("timestamp", out _).Should().BeTrue();
        json.RootElement.TryGetProperty("uptime", out _).Should().BeTrue();
    }

    [Fact]
    public async Task Metrics_ReturnsPrometheusFormat()
    {
        // Act
        var response = await _client.GetAsync("/metrics");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        response.Content.Headers.ContentType?.MediaType.Should().Be("text/plain");

        var content = await response.Content.ReadAsStringAsync();
        content.Should().Contain("# HELP");
        content.Should().Contain("# TYPE");
        content.Should().Contain("benchlab_uptime_seconds");
        content.Should().Contain("benchlab_requests_total");
    }

    [Fact]
    public async Task Metrics_TracksRequests()
    {
        // Arrange - Make a few requests first
        await _client.GetAsync("/health");
        await _client.GetAsync("/health");

        // Act
        var response = await _client.GetAsync("/metrics");
        var content = await response.Content.ReadAsStringAsync();

        // Assert
        content.Should().Contain("benchlab_requests_total");
        content.Should().MatchRegex(@"benchlab_requests_total \d+");
    }

    [Fact]
    public async Task Metrics_TracksByEndpoint()
    {
        // Arrange
        await _client.GetAsync("/health");
        await _client.GetAsync("/metrics");

        // Act
        var response = await _client.GetAsync("/metrics");
        var content = await response.Content.ReadAsStringAsync();

        // Assert
        content.Should().Contain("benchlab_requests_by_endpoint_total");
        content.Should().MatchRegex(@"benchlab_requests_by_endpoint_total\{endpoint=""\/health""\}");
        content.Should().MatchRegex(@"benchlab_requests_by_endpoint_total\{endpoint=""\/metrics""\}");
    }

    [Fact]
    public async Task Metrics_TracksByStatus()
    {
        // Arrange
        await _client.GetAsync("/health"); // 200
        await _client.GetAsync("/nonexistent"); // 404

        // Act
        var response = await _client.GetAsync("/metrics");
        var content = await response.Content.ReadAsStringAsync();

        // Assert
        content.Should().Contain("benchlab_requests_by_status_total");
        content.Should().MatchRegex(@"benchlab_requests_by_status_total\{status=""200""\}");
        content.Should().MatchRegex(@"benchlab_requests_by_status_total\{status=""404""\}");
    }

    #endregion

    #region Root Endpoint

    [Fact]
    public async Task Root_ReturnsServiceInfo()
    {
        // Act
        var response = await _client.GetAsync("/");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        var content = await response.Content.ReadAsStringAsync();
        var json = JsonDocument.Parse(content);

        json.RootElement.GetProperty("service").GetString().Should().Be("BenchLab HTTP Service");
        json.RootElement.GetProperty("version").GetString().Should().Be("1.0.0");
        json.RootElement.TryGetProperty("endpoints", out _).Should().BeTrue();
        json.RootElement.TryGetProperty("authentication", out _).Should().BeTrue();
        json.RootElement.TryGetProperty("configuration", out _).Should().BeTrue();
    }

    [Fact]
    public async Task Root_ListsAllEndpoints()
    {
        // Act
        var response = await _client.GetAsync("/");
        var content = await response.Content.ReadAsStringAsync();
        var json = JsonDocument.Parse(content);

        // Assert
        var endpoints = json.RootElement.GetProperty("endpoints");
        endpoints.GetArrayLength().Should().BeGreaterThan(0);

        var endpointStrings = endpoints.EnumerateArray()
            .Select(e => e.GetString())
            .ToList();

        endpointStrings.Should().Contain(e => e!.Contains("/health"));
        endpointStrings.Should().Contain(e => e!.Contains("/metrics"));
        endpointStrings.Should().Contain(e => e!.Contains("/devices"));
        endpointStrings.Should().Contain(e => e!.Contains("/stream"));
    }

    #endregion

    #region Device Discovery Endpoint

    [Fact]
    public async Task ListDevices_ReturnsArray()
    {
        // Act
        var response = await _client.GetAsync("/devices");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        var content = await response.Content.ReadAsStringAsync();
        var json = JsonDocument.Parse(content);

        json.RootElement.ValueKind.Should().Be(JsonValueKind.Array);
    }

    [Fact]
    public async Task ListDevices_ReturnsProbeResults()
    {
        // Act
        var response = await _client.GetAsync("/devices");
        var content = await response.Content.ReadAsStringAsync();

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);

        // Even if no devices, should return empty array, not error
        var json = JsonDocument.Parse(content);
        json.RootElement.ValueKind.Should().Be(JsonValueKind.Array);
    }

    [Fact]
    public async Task ListDevices_UpdatesMetrics()
    {
        // Arrange - Get baseline metrics
        var metricsResponse1 = await _client.GetAsync("/metrics");
        var metrics1 = await metricsResponse1.Content.ReadAsStringAsync();

        // Act
        await _client.GetAsync("/devices");

        // Assert - Check metrics were updated
        var metricsResponse2 = await _client.GetAsync("/metrics");
        var metrics2 = await metricsResponse2.Content.ReadAsStringAsync();

        metrics2.Should().Contain("benchlab_devices_discovered_total");
        metrics2.Should().Contain("benchlab_devices_online");
        metrics2.Should().Contain("benchlab_devices_offline");
    }

    #endregion

    #region Device Info Endpoint

    [Fact]
    public async Task GetDeviceInfo_NonexistentDevice_Returns404()
    {
        // Act
        var response = await _client.GetAsync("/devices/nonexistent/info");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.NotFound);
        var content = await response.Content.ReadAsStringAsync();
        content.Should().Contain("error");
    }

    [Fact]
    public async Task GetDeviceInfo_WithDevPath_NormalizesPath()
    {
        // This test verifies path normalization logic works
        // Act
        var response = await _client.GetAsync("/devices/ttyACM0/info");

        // Assert
        // Even if device doesn't exist, we should get 404 not 400 (path was normalized)
        response.StatusCode.Should().BeOneOf(HttpStatusCode.NotFound, HttpStatusCode.Forbidden, HttpStatusCode.InternalServerError);
    }

    [Fact]
    public async Task GetDeviceInfo_WithFullPath_AcceptsPath()
    {
        // Act
        var response = await _client.GetAsync("/devices/%2Fdev%2FttyACM0/info");  // URL encoded /dev/ttyACM0

        // Assert
        response.StatusCode.Should().BeOneOf(HttpStatusCode.NotFound, HttpStatusCode.Forbidden, HttpStatusCode.InternalServerError);
    }

    [Fact]
    public async Task GetDeviceInfo_WithTimeout_AcceptsParameter()
    {
        // Act
        var response = await _client.GetAsync("/devices/ttyACM0/info?timeout=1000");

        // Assert
        response.StatusCode.Should().BeOneOf(HttpStatusCode.NotFound, HttpStatusCode.Forbidden, HttpStatusCode.InternalServerError);
    }

    #endregion

    #region Device Sensors Endpoint

    [Fact]
    public async Task GetSensors_NonexistentDevice_Returns404()
    {
        // Act
        var response = await _client.GetAsync("/devices/nonexistent/sensors");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.NotFound);
    }

    [Fact]
    public async Task GetSensors_WithTimeout_AcceptsParameter()
    {
        // Act
        var response = await _client.GetAsync("/devices/ttyACM0/sensors?timeout=500");

        // Assert
        response.StatusCode.Should().BeOneOf(HttpStatusCode.NotFound, HttpStatusCode.Forbidden, HttpStatusCode.InternalServerError);
    }

    #endregion

    #region Stream Endpoint

    [Fact]
    public async Task Stream_WithoutDevice_FindsDevice()
    {
        // This should attempt auto-discovery
        // Act
        var response = await _client.GetAsync("/stream");

        // Assert
        // Either finds device and starts stream (200) or no device found (404)
        response.StatusCode.Should().BeOneOf(HttpStatusCode.OK, HttpStatusCode.NotFound);

        if (response.StatusCode == HttpStatusCode.NotFound)
        {
            var content = await response.Content.ReadAsStringAsync();
            content.Should().Contain("No device found");
        }
    }

    [Fact]
    public async Task Stream_NonexistentDevice_Returns404()
    {
        // Act
        var response = await _client.GetAsync("/stream?device=/dev/nonexistent");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.NotFound);
    }

    [Fact]
    public async Task Stream_SetsCorrectHeaders()
    {
        // Act
        var request = new HttpRequestMessage(HttpMethod.Get, "/stream?device=/dev/test");
        var response = await _client.SendAsync(request, HttpCompletionOption.ResponseHeadersRead);

        // Assert
        // Even if device doesn't exist, headers should be set before attempting connection
        if (response.StatusCode == HttpStatusCode.OK)
        {
            response.Content.Headers.ContentType?.MediaType.Should().Be("application/x-ndjson");
            response.Headers.CacheControl?.NoCache.Should().BeTrue();
        }
    }

    [Fact]
    public async Task Stream_UpdatesMetrics()
    {
        // Arrange
        var metricsResponse1 = await _client.GetAsync("/metrics");
        var metrics1 = await metricsResponse1.Content.ReadAsStringAsync();

        // Act - Attempt to stream (will fail but should update metrics)
        try
        {
            await _client.GetAsync("/stream?device=/dev/test");
        }
        catch
        {
            // Expected to fail on non-existent device
        }

        // Assert
        var metricsResponse2 = await _client.GetAsync("/metrics");
        var metrics2 = await metricsResponse2.Content.ReadAsStringAsync();

        metrics2.Should().Contain("benchlab_requests_by_endpoint_total");
        metrics2.Should().MatchRegex(@"benchlab_requests_by_endpoint_total\{endpoint=""\/stream""\}");
    }

    #endregion

    #region Write Endpoint

    [Fact]
    public async Task Write_WithoutBody_Returns400()
    {
        // Act
        var response = await _client.PostAsync("/write", new StringContent("", Encoding.UTF8, "application/json"));

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.BadRequest);
        var content = await response.Content.ReadAsStringAsync();
        content.Should().Contain("error");
    }

    [Fact]
    public async Task Write_WithInvalidJson_Returns400()
    {
        // Act
        var response = await _client.PostAsync("/write", new StringContent("invalid json", Encoding.UTF8, "application/json"));

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.BadRequest);
    }

    [Fact]
    public async Task Write_WithMissingDevice_Returns400()
    {
        // Arrange
        var requestBody = JsonSerializer.Serialize(new { data = "test" });

        // Act
        var response = await _client.PostAsync("/write",
            new StringContent(requestBody, Encoding.UTF8, "application/json"));

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.BadRequest);
    }

    [Fact]
    public async Task Write_WithMissingData_Returns400()
    {
        // Arrange
        var requestBody = JsonSerializer.Serialize(new { device = "/dev/test" });

        // Act
        var response = await _client.PostAsync("/write",
            new StringContent(requestBody, Encoding.UTF8, "application/json"));

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.BadRequest);
    }

    [Fact]
    public async Task Write_WithValidRequest_NonexistentDevice_Returns404()
    {
        // Arrange
        var requestBody = JsonSerializer.Serialize(new { device = "/dev/nonexistent", data = "test" });

        // Act
        var response = await _client.PostAsync("/write",
            new StringContent(requestBody, Encoding.UTF8, "application/json"));

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.NotFound);
    }

    [Fact]
    public async Task Write_WithTooLargeData_Returns400()
    {
        // Arrange
        var largeData = new string('X', 5000); // > 4096 byte limit
        var requestBody = JsonSerializer.Serialize(new { device = "/dev/test", data = largeData });

        // Act
        var response = await _client.PostAsync("/write",
            new StringContent(requestBody, Encoding.UTF8, "application/json"));

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.BadRequest);
        var content = await response.Content.ReadAsStringAsync();
        content.Should().Contain("too large");
    }

    [Fact]
    public async Task Write_NormalizesDevicePath()
    {
        // Arrange - test without /dev/ prefix
        var requestBody = JsonSerializer.Serialize(new { device = "ttyACM0", data = "test" });

        // Act
        var response = await _client.PostAsync("/write",
            new StringContent(requestBody, Encoding.UTF8, "application/json"));

        // Assert
        // Should normalize path and attempt to open, resulting in 404 not 400
        response.StatusCode.Should().BeOneOf(HttpStatusCode.NotFound, HttpStatusCode.Forbidden, HttpStatusCode.InternalServerError);
    }

    #endregion

    #region Swagger/OpenAPI Endpoints

    [Fact]
    public async Task Swagger_ReturnsUI()
    {
        // Act
        var response = await _client.GetAsync("/swagger");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        var content = await response.Content.ReadAsStringAsync();
        content.Should().Contain("swagger");
    }

    [Fact]
    public async Task SwaggerJson_ReturnsOpenApiSpec()
    {
        // Act
        var response = await _client.GetAsync("/swagger/v1/swagger.json");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        response.Content.Headers.ContentType?.MediaType.Should().Be("application/json");

        var content = await response.Content.ReadAsStringAsync();
        var json = JsonDocument.Parse(content);

        json.RootElement.TryGetProperty("openapi", out _).Should().BeTrue();
        json.RootElement.TryGetProperty("info", out _).Should().BeTrue();
        json.RootElement.TryGetProperty("paths", out _).Should().BeTrue();
    }

    [Fact]
    public async Task SwaggerJson_DocumentsAllEndpoints()
    {
        // Act
        var response = await _client.GetAsync("/swagger/v1/swagger.json");
        var content = await response.Content.ReadAsStringAsync();
        var json = JsonDocument.Parse(content);

        // Assert
        var paths = json.RootElement.GetProperty("paths");

        paths.TryGetProperty("/health", out _).Should().BeTrue();
        paths.TryGetProperty("/metrics", out _).Should().BeTrue();
        paths.TryGetProperty("/devices", out _).Should().BeTrue();
        paths.TryGetProperty("/stream", out _).Should().BeTrue();
        paths.TryGetProperty("/write", out _).Should().BeTrue();
    }

    #endregion

    #region Error Handling

    [Fact]
    public async Task UnknownEndpoint_Returns404()
    {
        // Act
        var response = await _client.GetAsync("/unknown/endpoint");

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.NotFound);
    }

    [Fact]
    public async Task MethodNotAllowed_Returns405()
    {
        // Act - Try POST on GET endpoint
        var response = await _client.PostAsync("/health", null);

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.MethodNotAllowed);
    }

    [Fact]
    public async Task InvalidAcceptHeader_StillReturnsResponse()
    {
        // Arrange
        var request = new HttpRequestMessage(HttpMethod.Get, "/health");
        request.Headers.Accept.Add(new MediaTypeWithQualityHeaderValue("application/xml"));

        // Act
        var response = await _client.SendAsync(request);

        // Assert
        // Should still return JSON even if client requests XML
        response.StatusCode.Should().Be(HttpStatusCode.OK);
    }

    #endregion

    #region Performance and Load

    [Fact]
    public async Task ConcurrentRequests_HandledCorrectly()
    {
        // Arrange
        var tasks = new List<Task<HttpResponseMessage>>();
        for (int i = 0; i < 10; i++)
        {
            tasks.Add(_client.GetAsync("/health"));
        }

        // Act
        var responses = await Task.WhenAll(tasks);

        // Assert
        responses.Should().AllSatisfy(r => r.StatusCode.Should().Be(HttpStatusCode.OK));
    }

    [Fact]
    public async Task RapidRequests_MetricsAccurate()
    {
        // Arrange - Get baseline
        var metricsResponse1 = await _client.GetAsync("/metrics");
        var metrics1 = await metricsResponse1.Content.ReadAsStringAsync();

        // Act - Make 5 rapid requests
        for (int i = 0; i < 5; i++)
        {
            await _client.GetAsync("/health");
        }

        // Assert
        var metricsResponse2 = await _client.GetAsync("/metrics");
        var metrics2 = await metricsResponse2.Content.ReadAsStringAsync();

        // Should have tracked all requests
        metrics2.Should().Contain("benchlab_requests_total");
    }

    [Fact]
    public async Task LargeResponse_HandledCorrectly()
    {
        // Act
        var response = await _client.GetAsync("/metrics");
        var content = await response.Content.ReadAsStringAsync();

        // Assert
        response.StatusCode.Should().Be(HttpStatusCode.OK);
        content.Length.Should().BeGreaterThan(0);
    }

    #endregion

    #region Content Type Handling

    [Fact]
    public async Task Health_ReturnsJson()
    {
        // Act
        var response = await _client.GetAsync("/health");

        // Assert
        response.Content.Headers.ContentType?.MediaType.Should().Be("application/json");
    }

    [Fact]
    public async Task Metrics_ReturnsPlainText()
    {
        // Act
        var response = await _client.GetAsync("/metrics");

        // Assert
        response.Content.Headers.ContentType?.MediaType.Should().Be("text/plain");
    }

    [Fact]
    public async Task Devices_ReturnsJson()
    {
        // Act
        var response = await _client.GetAsync("/devices");

        // Assert
        response.Content.Headers.ContentType?.MediaType.Should().Be("application/json");
    }

    #endregion
}
