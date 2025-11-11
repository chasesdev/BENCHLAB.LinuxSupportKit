using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using BenchLab.Platform.Discovery;
using BenchLab.Platform.Ports;
using BenchLab.Platform.Protocol;
using Microsoft.Extensions.Logging;

var logFactory = LoggerFactory.Create(builder => builder.AddConsole().SetMinimumLevel(LogLevel.Information));
var log = logFactory.CreateLogger("benchlab-cli");

static void Help()
{
    Console.WriteLine("benchlab-cli commands:");
    Console.WriteLine("  list [--timeout ms]                           - List available BenchLab devices");
    Console.WriteLine("  info --device PATH [--timeout ms]             - Show device information");
    Console.WriteLine("  sensors --device PATH [--timeout ms]          - Read sensor telemetry (one-shot)");
    Console.WriteLine("  stream [--device PATH] [--timeout ms] [--raw] - Stream telemetry data");
    Console.WriteLine("  write --device PATH --text "STRING"          - Send ASCII data to device");
}

if (args.Length == 0) { Help(); return; }

var cmd = args[0];
string? device = null;
int timeoutMs = 600;
bool raw = false;
string? text = null;

for (int i = 1; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--device": device = args[++i]; break;
        case "--timeout": timeoutMs = int.Parse(args[++i]); break;
        case "--raw": raw = true; break;
        case "--text": text = args[++i]; break;
        case "-h":
        case "--help": Help(); return;
    }
}

if (cmd == "list")
{
    var d = new PortDiscovery(new BenchlabHandshake(validateVendorData: true, log: log), log);
    var results = d.Discover(TimeSpan.FromMilliseconds(timeoutMs)).ToArray();
    Console.WriteLine(JsonSerializer.Serialize(results, new JsonSerializerOptions { WriteIndented = true }));
    return;
}
else if (cmd == "info")
{
    await InfoAsync();
    return;

    async Task InfoAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli info --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var vendorData = await protocol.ReadVendorDataAsync();
            var deviceName = await protocol.ReadDeviceNameAsync();

            var info = new
            {
                device,
                name = deviceName,
                vendorId = $"0x{vendorData.VendorId:X2}",
                productId = $"0x{vendorData.ProductId:X2}",
                firmwareVersion = vendorData.FwVersion,
                isValid = vendorData.IsValid
            };

            Console.WriteLine(JsonSerializer.Serialize(info, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to read device info");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "sensors")
{
    await SensorsAsync();
    return;

    async Task SensorsAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli sensors --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            unsafe
            {
                var sensors = await protocol.ReadSensorsAsync();

                var data = new
                {
                    device,
                    timestamp = DateTime.UtcNow.ToString("o"),
                    voltages = sensors.GetVoltages().Select((v, i) => new { channel = i, millivolts = v, volts = v / 1000.0 }).ToArray(),
                    temperatures = new
                    {
                        chip = sensors.ChipTemperature,
                        ambient = sensors.AmbientTemperature,
                        sensors = sensors.GetTemperatures()
                    },
                    humidity = sensors.HumidityPercent,
                    power = sensors.GetPowerReadings().Select((p, i) => new
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

                Console.WriteLine(JsonSerializer.Serialize(data, new JsonSerializerOptions { WriteIndented = true }));
            }
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to read sensors");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "stream")
{
    await StreamAsync();
    return;

    async Task StreamAsync()
    {
        var d = new PortDiscovery(new BenchlabHandshake(validateVendorData: true, log: log), log);
        var results = d.Discover(TimeSpan.FromMilliseconds(timeoutMs)).ToArray();
        var target = device ?? results.FirstOrDefault(r => r.IsBenchlab)?.Device ?? results.FirstOrDefault()?.Device;
        if (target is null) { Console.Error.WriteLine("no device found"); Environment.Exit(2); }
        log.LogInformation("Streaming from {Device}", target);

        using var sp = SerialPortAdapter.Open(target, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
        var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs));
        using var cts = new CancellationTokenSource();

        // Handle Ctrl+C gracefully
        Console.CancelKeyPress += (sender, e) =>
        {
            e.Cancel = true;
            cts.Cancel();
        };

        // Stream sensor readings continuously using event-driven async I/O
        while (!cts.Token.IsCancellationRequested)
        {
            try
            {
                unsafe
                {
                    var sensors = await protocol.ReadSensorsAsync(cts.Token);
                    var data = new
                    {
                        device = target,
                        timestamp = DateTime.UtcNow.ToString("o"),
                        voltages = sensors.GetVoltages(),
                        chipTemp = sensors.ChipTemperature,
                        ambientTemp = sensors.AmbientTemperature,
                        humidity = sensors.HumidityPercent,
                        power = sensors.GetPowerReadings().Select(p => new { v = p.VoltageVolts, a = p.CurrentAmps, w = p.PowerWatts }).ToArray(),
                        fans = sensors.GetFans().Select(f => new { enabled = f.IsEnabled, duty = f.DutyPercent, rpm = f.Tach }).ToArray()
                    };

                    Console.WriteLine(JsonSerializer.Serialize(data));
                }

                // Delay between reads for reasonable update rate (~10Hz)
                await Task.Delay(100, cts.Token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                log.LogError(ex, "stream error");
                try { await Task.Delay(1000, cts.Token); } catch { break; }
            }
        }
    }
}
else if (cmd == "write")
{
    if (string.IsNullOrEmpty(device) || text is null)
    {
        Console.Error.WriteLine("usage: benchlab-cli write --device PATH --text "STRING"");
        Environment.Exit(2);
    }
    using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr:true, rts:true);
    var bytes = Encoding.ASCII.GetBytes(text);
    sp.Write(bytes, 0, bytes.Length);
    Console.WriteLine("ok");
    return;
}
else
{
    Help();
}
