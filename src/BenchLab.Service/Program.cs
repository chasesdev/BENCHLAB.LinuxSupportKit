using System.Text;
using System.Text.Json;
using BenchLab.Platform.Discovery;
using BenchLab.Platform.Ports;

var builder = WebApplication.CreateBuilder(args);
var app = builder.Build();

var handshake = new SimpleHandshake(ping: null);
var discovery = new PortDiscovery(handshake);

app.MapGet("/devices", () =>
{
    var results = discovery.Discover(TimeSpan.FromMilliseconds(600)).ToArray();
    return Results.Json(results);
});

app.MapGet("/stream", async (HttpContext ctx) =>
{
    var device = ctx.Request.Query["device"].ToString();
    if (string.IsNullOrEmpty(device))
    {
        var first = discovery.Discover(TimeSpan.FromMilliseconds(600)).FirstOrDefault(r => r.IsBenchlab)?.Device;
        device = first ?? discovery.Discover(TimeSpan.FromMilliseconds(600)).FirstOrDefault()?.Device ?? "";
    }
    if (string.IsNullOrEmpty(device)) return Results.BadRequest("no device");

    ctx.Response.Headers.CacheControl = "no-cache";
    ctx.Response.ContentType = "application/x-ndjson";

    using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(600), TimeSpan.FromMilliseconds(600), dtr:true, rts:true);
    var buf = new byte[256];
    while (!ctx.RequestAborted.IsCancellationRequested)
    {
        try
        {
            if (sp.BytesToRead > 0)
            {
                var n = sp.Read(buf, 0, Math.Min(buf.Length, sp.BytesToRead));
                if (n > 0)
                {
                    var hex = BitConverter.ToString(buf, 0, n).Replace("-", " ");
                    var payload = JsonSerializer.Serialize(new { device, hex, ts = DateTime.UtcNow });
                    await ctx.Response.WriteAsync(payload + "\n");
                    await ctx.Response.Body.FlushAsync();
                }
            }
            await Task.Delay(10, ctx.RequestAborted);
        }
        catch (TaskCanceledException) { break; }
        catch (Exception ex)
        {
            var payload = JsonSerializer.Serialize(new { device, error = ex.Message, ts = DateTime.UtcNow });
            await ctx.Response.WriteAsync(payload + "\n");
            await Task.Delay(200, ctx.RequestAborted);
        }
    }
    return Results.Empty;
});

app.MapPost("/write", async (HttpContext ctx) =>
{
    using var reader = new StreamReader(ctx.Request.Body, Encoding.UTF8);
    var body = await reader.ReadToEndAsync();
    var obj = JsonSerializer.Deserialize<WriteRequest>(body);
    if (obj is null || string.IsNullOrEmpty(obj.Device) || obj.Data is null)
        return Results.BadRequest("body: { device: '/dev/ttyACM0', data: 'STRING' }");

    using var sp = SerialPortAdapter.Open(obj.Device, 115200, TimeSpan.FromMilliseconds(600), TimeSpan.FromMilliseconds(600), dtr:true, rts:true);
    var bytes = Encoding.ASCII.GetBytes(obj.Data);
    sp.Write(bytes, 0, bytes.Length);
    return Results.Ok(new { ok = true });
});

app.Run("http://0.0.0.0:8080");

record WriteRequest(string Device, string Data);
