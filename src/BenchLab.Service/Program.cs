using System.Diagnostics;
using System.Text;
using System.Text.Json;
using BenchLab.Platform.Discovery;
using BenchLab.Platform.Ports;
using BenchLab.Platform.Protocol;
using BenchLab.Service.Metrics;
using Microsoft.AspNetCore.Mvc;

// Configuration
var builder = WebApplication.CreateBuilder(args);

// Get configuration values with environment variable overrides
var bindAddress = Environment.GetEnvironmentVariable("BENCHLAB_BIND_ADDRESS") ?? "http://127.0.0.1:8080";
var apiKey = Environment.GetEnvironmentVariable("BENCHLAB_API_KEY");
var discoveryTimeoutMs = int.Parse(Environment.GetEnvironmentVariable("BENCHLAB_DISCOVERY_TIMEOUT_MS") ?? "600");
var commandTimeoutMs = int.Parse(Environment.GetEnvironmentVariable("BENCHLAB_COMMAND_TIMEOUT_MS") ?? "500");

// Enable OpenAPI/Swagger
builder.Services.AddEndpointsApiExplorer();
builder.Services.AddSwaggerGen(options =>
{
    options.SwaggerDoc("v1", new() { Title = "BenchLab HTTP Service", Version = "v1.0", Description = "REST API for BenchLab device communication" });
});

// Configure logging
builder.Logging.ClearProviders();
builder.Logging.AddConsole();
builder.Logging.AddFilter("Microsoft.AspNetCore", LogLevel.Warning);

// Register services
builder.Services.AddSingleton<MetricsCollector>();

builder.Services.AddSingleton<IBenchlabHandshake>(sp =>
{
    var logger = sp.GetRequiredService<ILoggerFactory>().CreateLogger("BenchlabHandshake");
    return new BenchlabHandshake(validateVendorData: true, log: logger);
});

builder.Services.AddSingleton<PortDiscovery>(sp =>
{
    var handshake = sp.GetRequiredService<IBenchlabHandshake>();
    var logger = sp.GetRequiredService<ILoggerFactory>().CreateLogger("PortDiscovery");
    return new PortDiscovery(handshake, logger);
});

var app = builder.Build();

var metrics = app.Services.GetRequiredService<MetricsCollector>();

// Middleware: Request metrics tracking
app.Use(async (context, next) =>
{
    var sw = Stopwatch.StartNew();
    await next();
    sw.Stop();

    var endpoint = context.Request.Path.ToString();
    var statusCode = context.Response.StatusCode;
    metrics.RecordRequest(endpoint, statusCode, sw.Elapsed);
});

// Middleware: API Key Authentication
app.Use(async (context, next) =>
{
    // Skip auth for health/metrics endpoints
    if (context.Request.Path.StartsWithSegments("/health") ||
        context.Request.Path.StartsWithSegments("/metrics") ||
        context.Request.Path.StartsWithSegments("/swagger") ||
        context.Request.Path.StartsWithSegments("/"))
    {
        await next();
        return;
    }

    if (!string.IsNullOrEmpty(apiKey))
    {
        var authHeader = context.Request.Headers.Authorization.ToString();
        if (!authHeader.StartsWith("Bearer ") || authHeader.Substring(7) != apiKey)
        {
            context.Response.StatusCode = 401;
            await context.Response.WriteAsJsonAsync(new { error = "Unauthorized. Provide valid API key in Authorization: Bearer header." });
            return;
        }
    }

    await next();
});

// Enable Swagger UI
app.UseSwagger();
app.UseSwaggerUI();

var logger = app.Logger;
var discovery = app.Services.GetRequiredService<PortDiscovery>();

// Root endpoint - API info
app.MapGet("/", () => Results.Json(new
{
    service = "BenchLab HTTP Service",
    version = "1.0.0",
    endpoints = new[]
    {
        "GET /health - Health check",
        "GET /metrics - Service metrics",
        "GET /devices - List available devices",
        "GET /devices/{id}/info - Get device information",
        "GET /devices/{id}/sensors - Read sensor telemetry",
        "GET /stream?device=/dev/benchlab0 - Stream telemetry (NDJSON)",
        "POST /write - Write data to device"
    },
    authentication = string.IsNullOrEmpty(apiKey) ? "disabled" : "enabled (API key required)",
    configuration = new
    {
        bindAddress,
        discoveryTimeoutMs,
        commandTimeoutMs
    }
}))
.WithName("GetInfo")
.WithTags("Info")
.WithOpenApi();

// Health check endpoint
app.MapGet("/health", () =>
{
    return Results.Json(new
    {
        status = "healthy",
        timestamp = DateTime.UtcNow.ToString("o"),
        uptime = Environment.TickCount64 / 1000.0
    });
})
.WithName("HealthCheck")
.WithTags("Monitoring")
.WithOpenApi(operation => { operation.Summary = "Health check endpoint"; return operation; });

// Metrics endpoint (Prometheus-compatible format)
app.MapGet("/metrics", () =>
{
    var output = metrics.ExportPrometheusFormat();
    return Results.Text(output, "text/plain; version=0.0.4");
})
.WithName("Metrics")
.WithTags("Monitoring")
.WithOpenApi(operation => { operation.Summary = "Prometheus metrics"; return operation; });

// List all devices
app.MapGet("/devices", () =>
{
    try
    {
        var results = discovery.Discover(TimeSpan.FromMilliseconds(discoveryTimeoutMs)).ToArray();
        var benchlabCount = results.Count(r => r.IsBenchlab);

        metrics.RecordDeviceDiscovery(results.Length, benchlabCount);

        logger.LogInformation("Discovered {Count} devices ({BenchLab} BenchLab devices)",
            results.Length, benchlabCount);
        return Results.Json(results);
    }
    catch (Exception ex)
    {
        logger.LogError(ex, "Failed to discover devices");
        return Results.Problem(detail: ex.Message, statusCode: 500, title: "Discovery failed");
    }
})
.WithName("ListDevices")
.WithTags("Devices")
.WithOpenApi(operation => { operation.Summary = "List all available devices"; return operation; });

// Get device information
app.MapGet("/devices/{id}/info", async (string id, [FromQuery] int? timeout) =>
{
    var devicePath = id.StartsWith("/dev/") ? id : $"/dev/{id}";
    var timeoutMs = timeout ?? commandTimeoutMs;

    try
    {
        using var sp = SerialPortAdapter.Open(devicePath, 115200,
            TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs),
            dtr: true, rts: true);
        var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), logger);

        var vendorData = await protocol.ReadVendorDataAsync();
        metrics.RecordProtocolCommand("ReadVendorData", true);

        var deviceName = await protocol.ReadDeviceNameAsync();
        metrics.RecordProtocolCommand("ReadDeviceName", true);

        var info = new
        {
            device = devicePath,
            name = deviceName,
            vendorId = $"0x{vendorData.VendorId:X2}",
            productId = $"0x{vendorData.ProductId:X2}",
            firmwareVersion = vendorData.FwVersion,
            isValid = vendorData.IsValid,
            timestamp = DateTime.UtcNow.ToString("o")
        };

        return Results.Json(info);
    }
    catch (UnauthorizedAccessException ex)
    {
        metrics.RecordProtocolCommand("ReadVendorData", false);
        logger.LogError(ex, "Access denied to {Device}", devicePath);
        return Results.Problem(detail: "Access denied to device", statusCode: 403, title: "Access Denied");
    }
    catch (FileNotFoundException ex)
    {
        metrics.RecordProtocolCommand("ReadVendorData", false);
        logger.LogError(ex, "Device not found: {Device}", devicePath);
        return Results.NotFound(new { error = "Device not found", device = devicePath });
    }
    catch (TimeoutException ex)
    {
        metrics.RecordProtocolCommand("ReadVendorData", false);
        logger.LogError(ex, "Timeout reading {Device}", devicePath);
        return Results.Problem(detail: "Device did not respond in time", statusCode: 504, title: "Timeout");
    }
    catch (Exception ex)
    {
        metrics.RecordProtocolCommand("ReadVendorData", false);
        logger.LogError(ex, "Failed to read device info from {Device}", devicePath);
        return Results.Problem(detail: ex.Message, statusCode: 500, title: "Device communication failed");
    }
})
.WithName("GetDeviceInfo")
.WithTags("Devices")
.WithOpenApi(operation => { operation.Summary = "Get device information"; return operation; });

// Read sensor telemetry (one-shot)
app.MapGet("/devices/{id}/sensors", async (string id, [FromQuery] int? timeout) =>
{
    var devicePath = id.StartsWith("/dev/") ? id : $"/dev/{id}";
    var timeoutMs = timeout ?? commandTimeoutMs;

    try
    {
        using var sp = SerialPortAdapter.Open(devicePath, 115200,
            TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs),
            dtr: true, rts: true);
        var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), logger);

        unsafe
        {
            var sensors = await protocol.ReadSensorsAsync();
            metrics.RecordProtocolCommand("ReadSensors", true);

            var powerReadings = sensors.GetPowerReadings();
            var totalPower = powerReadings.Sum(p => p.PowerWatts);
            var fanSpeeds = sensors.GetFans().Select(f => f.Tach).ToArray();

            metrics.RecordDeviceTelemetry(devicePath, sensors.ChipTemperature, sensors.AmbientTemperature,
                totalPower, fanSpeeds);

            var data = new
            {
                device = devicePath,
                timestamp = DateTime.UtcNow.ToString("o"),
                voltages = sensors.GetVoltages().Select((v, i) => new { channel = i, millivolts = v, volts = v / 1000.0 }).ToArray(),
                temperatures = new
                {
                    chip = sensors.ChipTemperature,
                    ambient = sensors.AmbientTemperature,
                    sensors = sensors.GetTemperatures()
                },
                humidity = sensors.HumidityPercent,
                power = powerReadings.Select((p, i) => new
                {
                    channel = i,
                    voltage = p.VoltageVolts,
                    current = p.CurrentAmps,
                    power = p.PowerWatts
                }).ToArray(),
                fans = sensors.GetFans().Select((f, i) => new
                {
                    channel = i,
                    enabled = f.IsEnabled,
                    duty = f.DutyPercent,
                    rpm = f.Tach
                }).ToArray()
            };

            return Results.Json(data);
        }
    }
    catch (UnauthorizedAccessException ex)
    {
        metrics.RecordProtocolCommand("ReadSensors", false);
        logger.LogError(ex, "Access denied to {Device}", devicePath);
        return Results.Problem(detail: "Access denied to device", statusCode: 403, title: "Access Denied");
    }
    catch (FileNotFoundException ex)
    {
        metrics.RecordProtocolCommand("ReadSensors", false);
        logger.LogError(ex, "Device not found: {Device}", devicePath);
        return Results.NotFound(new { error = "Device not found", device = devicePath });
    }
    catch (TimeoutException ex)
    {
        metrics.RecordProtocolCommand("ReadSensors", false);
        logger.LogError(ex, "Timeout reading {Device}", devicePath);
        return Results.Problem(detail: "Device did not respond in time", statusCode: 504, title: "Timeout");
    }
    catch (Exception ex)
    {
        metrics.RecordProtocolCommand("ReadSensors", false);
        logger.LogError(ex, "Failed to read sensors from {Device}", devicePath);
        return Results.Problem(detail: ex.Message, statusCode: 500, title: "Sensor read failed");
    }
})
.WithName("GetSensors")
.WithTags("Devices")
.WithOpenApi(operation => { operation.Summary = "Read sensor telemetry (one-shot)"; return operation; });

// Stream telemetry data (NDJSON)
app.MapGet("/stream", async (HttpContext ctx, [FromQuery] string? device, [FromQuery] int? timeout) =>
{
    var timeoutMs = timeout ?? commandTimeoutMs;
    string devicePath;

    if (string.IsNullOrEmpty(device))
    {
        var results = discovery.Discover(TimeSpan.FromMilliseconds(discoveryTimeoutMs)).ToArray();
        devicePath = results.FirstOrDefault(r => r.IsBenchlab)?.Device ?? results.FirstOrDefault()?.Device ?? "";
    }
    else
    {
        devicePath = device.StartsWith("/dev/") ? device : $"/dev/{device}";
    }

    if (string.IsNullOrEmpty(devicePath))
    {
        ctx.Response.StatusCode = 404;
        await ctx.Response.WriteAsJsonAsync(new { error = "No device found" });
        return;
    }

    ctx.Response.Headers.CacheControl = "no-cache";
    ctx.Response.ContentType = "application/x-ndjson";

    ISerialPort? sp = null;
    long bytesTransmitted = 0;
    bool hadError = false;

    try
    {
        sp = SerialPortAdapter.Open(devicePath, 115200,
            TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs),
            dtr: true, rts: true);
        var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), logger);

        metrics.RecordStreamStart();
        logger.LogInformation("Starting stream from {Device}", devicePath);

        // Pre-allocate reusable buffers outside loop (zero-allocation streaming)
        var powerBuffer = new PowerSensor[11];
        var fanBuffer = new FanSensor[9];
        var voltageBuffer = new short[13];
        var fanSpeedBuffer = new int[9];

        // Stream sensor readings continuously using event-driven async I/O
        while (!ctx.RequestAborted.IsCancellationRequested)
        {
            try
            {
                unsafe
                {
                    var sensors = await protocol.ReadSensorsAsync(ctx.RequestAborted);
                    metrics.RecordProtocolCommand("ReadSensors", true);

                    // Fill pre-allocated buffers (zero allocation)
                    sensors.FillPowerReadings(powerBuffer);
                    sensors.FillFans(fanBuffer);
                    sensors.FillVoltages(voltageBuffer);

                    // Calculate total power without LINQ
                    double totalPower = 0;
                    for (int i = 0; i < powerBuffer.Length; i++)
                        totalPower += powerBuffer[i].PowerWatts;

                    // Extract fan speeds
                    for (int i = 0; i < fanBuffer.Length; i++)
                        fanSpeedBuffer[i] = fanBuffer[i].Tach;

                    metrics.RecordDeviceTelemetry(devicePath, sensors.ChipTemperature, sensors.AmbientTemperature,
                        totalPower, fanSpeedBuffer);

                    // Build response data structure (still allocates anonymous object, but reduces intermediate allocations)
                    var data = new
                    {
                        device = devicePath,
                        timestamp = DateTime.UtcNow.ToString("o"),
                        voltages = voltageBuffer,
                        chipTemp = sensors.ChipTemperature,
                        ambientTemp = sensors.AmbientTemperature,
                        humidity = sensors.HumidityPercent,
                        power = BuildPowerArray(powerBuffer),
                        fans = BuildFanArray(fanBuffer)
                    };

                    var json = JsonSerializer.Serialize(data);
                    var bytes = Encoding.UTF8.GetByteCount(json) + 1; // +1 for newline
                    bytesTransmitted += bytes;

                    await ctx.Response.WriteAsync(json + "\n", ctx.RequestAborted);
                    await ctx.Response.Body.FlushAsync(ctx.RequestAborted);
                }

                // Delay between reads for reasonable update rate (~10Hz)
                await Task.Delay(100, ctx.RequestAborted);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                hadError = true;
                logger.LogError(ex, "Stream error from {Device}", devicePath);
                metrics.RecordProtocolCommand("ReadSensors", false);

                var error = new { device = devicePath, error = ex.Message, timestamp = DateTime.UtcNow.ToString("o") };
                var errorJson = JsonSerializer.Serialize(error) + "\n";
                bytesTransmitted += Encoding.UTF8.GetByteCount(errorJson);
                await ctx.Response.WriteAsync(errorJson, ctx.RequestAborted);

                try { await Task.Delay(1000, ctx.RequestAborted); } catch { break; }
            }
        }

        metrics.RecordStreamEnd(bytesTransmitted, hadError);
        logger.LogInformation("Stream ended for {Device}", devicePath);
    }
    catch (UnauthorizedAccessException)
    {
        ctx.Response.StatusCode = 403;
        await ctx.Response.WriteAsJsonAsync(new { error = "Access denied to device", device = devicePath });
    }
    catch (FileNotFoundException)
    {
        ctx.Response.StatusCode = 404;
        await ctx.Response.WriteAsJsonAsync(new { error = "Device not found", device = devicePath });
    }
    catch (IOException ex) when (ex.Message.Contains("in use"))
    {
        ctx.Response.StatusCode = 409;
        await ctx.Response.WriteAsJsonAsync(new { error = "Device is busy (already in use)", device = devicePath });
    }
    catch (Exception ex)
    {
        logger.LogError(ex, "Failed to open stream from {Device}", devicePath);
        ctx.Response.StatusCode = 500;
        await ctx.Response.WriteAsJsonAsync(new { error = ex.Message, device = devicePath });
    }
    finally
    {
        sp?.Dispose();
    }
})
.WithName("StreamTelemetry")
.WithTags("Streaming")
.WithOpenApi(operation => { operation.Summary = "Stream telemetry data (NDJSON format)"; return operation; });

// Write data to device
app.MapPost("/write", async (HttpContext ctx) =>
{
    try
    {
        using var reader = new StreamReader(ctx.Request.Body, Encoding.UTF8);
        var body = await reader.ReadToEndAsync();
        var obj = JsonSerializer.Deserialize<WriteRequest>(body);

        if (obj is null || string.IsNullOrEmpty(obj.Device) || obj.Data is null)
        {
            return Results.BadRequest(new { error = "Invalid request", example = new { device = "/dev/benchlab0", data = "command" } });
        }

        // Validate device path
        var devicePath = obj.Device.StartsWith("/dev/") ? obj.Device : $"/dev/{obj.Device}";

        // Limit data size to prevent abuse
        if (obj.Data.Length > 4096)
        {
            return Results.BadRequest(new { error = "Data too large (max 4096 bytes)" });
        }

        using var sp = SerialPortAdapter.Open(devicePath, 115200,
            TimeSpan.FromMilliseconds(commandTimeoutMs), TimeSpan.FromMilliseconds(commandTimeoutMs),
            dtr: true, rts: true);

        var bytes = Encoding.UTF8.GetBytes(obj.Data);
        sp.Write(bytes, 0, bytes.Length);

        logger.LogInformation("Wrote {Bytes} bytes to {Device}", bytes.Length, devicePath);
        return Results.Ok(new { success = true, device = devicePath, bytesWritten = bytes.Length });
    }
    catch (UnauthorizedAccessException)
    {
        return Results.Problem(detail: "Access denied to device", statusCode: 403, title: "Access Denied");
    }
    catch (FileNotFoundException ex)
    {
        return Results.NotFound(new { error = "Device not found", detail = ex.Message });
    }
    catch (JsonException)
    {
        return Results.BadRequest(new { error = "Invalid JSON", example = new { device = "/dev/benchlab0", data = "command" } });
    }
    catch (Exception ex)
    {
        logger.LogError(ex, "Failed to write to device");
        return Results.Problem(detail: ex.Message, statusCode: 500, title: "Write failed");
    }
})
.WithName("WriteToDevice")
.WithTags("Devices")
.WithOpenApi(operation => { operation.Summary = "Write data to device"; return operation; });

// Log configuration on startup
logger.LogInformation("BenchLab HTTP Service starting");
logger.LogInformation("Bind address: {BindAddress}", bindAddress);
logger.LogInformation("API authentication: {Auth}", string.IsNullOrEmpty(apiKey) ? "DISABLED (no BENCHLAB_API_KEY set)" : "ENABLED");
logger.LogInformation("Discovery timeout: {Timeout}ms", discoveryTimeoutMs);
logger.LogInformation("Command timeout: {Timeout}ms", commandTimeoutMs);
logger.LogInformation("Swagger UI available at: {Url}/swagger", bindAddress);

if (string.IsNullOrEmpty(apiKey))
{
    logger.LogWarning("⚠️  WARNING: Running without authentication! Set BENCHLAB_API_KEY environment variable for production use.");
}

if (bindAddress.Contains("0.0.0.0"))
{
    logger.LogWarning("⚠️  WARNING: Binding to all interfaces (0.0.0.0). This exposes the service to your entire network!");
}

app.Run(bindAddress);

record WriteRequest(string Device, string Data);

// Helper methods for streaming optimization
static object[] BuildPowerArray(PowerSensor[] powerBuffer)
{
    var result = new object[powerBuffer.Length];
    for (int i = 0; i < powerBuffer.Length; i++)
    {
        result[i] = new
        {
            v = powerBuffer[i].VoltageVolts,
            a = powerBuffer[i].CurrentAmps,
            w = powerBuffer[i].PowerWatts
        };
    }
    return result;
}

static object[] BuildFanArray(FanSensor[] fanBuffer)
{
    var result = new object[fanBuffer.Length];
    for (int i = 0; i < fanBuffer.Length; i++)
    {
        result[i] = new
        {
            enabled = fanBuffer[i].IsEnabled,
            duty = fanBuffer[i].DutyPercent,
            rpm = fanBuffer[i].Tach
        };
    }
    return result;
}
