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
    Console.WriteLine("  list [--timeout ms]                              - List available BenchLab devices");
    Console.WriteLine("  info --device PATH [--timeout ms]                - Show device information");
    Console.WriteLine("  sensors --device PATH [--timeout ms]             - Read sensor telemetry (one-shot)");
    Console.WriteLine("  stream [--device PATH] [--timeout ms] [--raw]    - Stream telemetry data");
    Console.WriteLine("  write --device PATH --text "STRING"             - Send ASCII data to device");
    Console.WriteLine("");
    Console.WriteLine("Device Configuration:");
    Console.WriteLine("  set-name --device PATH --name "NAME"            - Set device name (max 32 chars)");
    Console.WriteLine("  get-uid --device PATH [--timeout ms]             - Get unique device ID (96-bit)");
    Console.WriteLine("");
    Console.WriteLine("RGB LED Control:");
    Console.WriteLine("  get-rgb --device PATH [--timeout ms]             - Get RGB LED settings");
    Console.WriteLine("  set-rgb --device PATH --mode MODE [--red R]      - Set RGB LED configuration");
    Console.WriteLine("          [--green G] [--blue B] [--brightness B]");
    Console.WriteLine("          [--speed S] [--timeout ms]");
    Console.WriteLine("          Modes: off, solid, breathing, cycle, temperature");
    Console.WriteLine("");
    Console.WriteLine("Fan Control:");
    Console.WriteLine("  get-fan --device PATH --fan INDEX [--timeout ms] - Get fan profile (0-8)");
    Console.WriteLine("  set-fan-manual --device PATH --fan INDEX         - Set fan to manual mode");
    Console.WriteLine("                 --duty DUTY [--timeout ms]");
    Console.WriteLine("  set-fan-auto --device PATH --fan INDEX           - Set fan to auto mode");
    Console.WriteLine("               --temp-threshold TEMP --min-duty MIN");
    Console.WriteLine("               --max-duty MAX [--sensor SENSOR]");
    Console.WriteLine("               [--timeout ms]");
    Console.WriteLine("");
    Console.WriteLine("Calibration:");
    Console.WriteLine("  calibration get --device PATH [--timeout ms]     - Get calibration data");
    Console.WriteLine("  calibration load --device PATH [--timeout ms]    - Load calibration from flash");
    Console.WriteLine("  calibration store --device PATH [--timeout ms]   - Save calibration to flash");
    Console.WriteLine("  calibration set --device PATH --json FILE        - Apply calibration from JSON");
    Console.WriteLine("                  [--timeout ms]");
    Console.WriteLine("");
    Console.WriteLine("Actions:");
    Console.WriteLine("  action --device PATH --id ACTION_ID              - Execute device action");
    Console.WriteLine("         [--timeout ms]");
}

if (args.Length == 0) { Help(); return; }

var cmd = args[0];
string? device = null;
int timeoutMs = 600;
bool raw = false;
string? text = null;
string? name = null;
string? mode = null;
int? red = null;
int? green = null;
int? blue = null;
int? brightness = null;
int? speed = null;
int? fan = null;
int? duty = null;
double? tempThreshold = null;
int? minDuty = null;
int? maxDuty = null;
int? sensor = null;
string? json = null;
int? actionId = null;
string? subcommand = null;

for (int i = 1; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--device": device = args[++i]; break;
        case "--timeout": timeoutMs = int.Parse(args[++i]); break;
        case "--raw": raw = true; break;
        case "--text": text = args[++i]; break;
        case "--name": name = args[++i]; break;
        case "--mode": mode = args[++i]; break;
        case "--red": red = int.Parse(args[++i]); break;
        case "--green": green = int.Parse(args[++i]); break;
        case "--blue": blue = int.Parse(args[++i]); break;
        case "--brightness": brightness = int.Parse(args[++i]); break;
        case "--speed": speed = int.Parse(args[++i]); break;
        case "--fan": fan = int.Parse(args[++i]); break;
        case "--duty": duty = int.Parse(args[++i]); break;
        case "--temp-threshold": tempThreshold = double.Parse(args[++i]); break;
        case "--min-duty": minDuty = int.Parse(args[++i]); break;
        case "--max-duty": maxDuty = int.Parse(args[++i]); break;
        case "--sensor": sensor = int.Parse(args[++i]); break;
        case "--json": json = args[++i]; break;
        case "--id": actionId = int.Parse(args[++i]); break;
        case "get":
        case "load":
        case "store":
        case "set": subcommand = args[i]; break;
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
else if (cmd == "set-name")
{
    await SetNameAsync();
    return;

    async Task SetNameAsync()
    {
        if (string.IsNullOrEmpty(device) || string.IsNullOrEmpty(name))
        {
            Console.Error.WriteLine("usage: benchlab-cli set-name --device PATH --name \"NAME\"");
            Environment.Exit(2);
        }

        if (name.Length > 32)
        {
            Console.Error.WriteLine("error: name must be 32 characters or less");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            await protocol.WriteDeviceNameAsync(name);
            Console.WriteLine(JsonSerializer.Serialize(new { status = "ok", name }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to set device name");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "get-uid")
{
    await GetUidAsync();
    return;

    async Task GetUidAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli get-uid --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var uid = await protocol.ReadDeviceUidAsync();
            var uidHex = uid.ToHexString();

            var result = new
            {
                device,
                uid = uidHex,
                uidLow = BitConverter.ToString(uid.UidLow).Replace("-", ""),
                uidHigh = BitConverter.ToString(uid.UidHigh).Replace("-", "")
            };

            Console.WriteLine(JsonSerializer.Serialize(result, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to read device UID");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "get-rgb")
{
    await GetRgbAsync();
    return;

    async Task GetRgbAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli get-rgb --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var rgb = await protocol.ReadRgbAsync();
            var modeStr = rgb.Mode switch
            {
                0 => "off",
                1 => "solid",
                2 => "breathing",
                3 => "cycle",
                4 => "temperature",
                _ => "unknown"
            };

            var result = new
            {
                device,
                mode = modeStr,
                red = rgb.Red,
                green = rgb.Green,
                blue = rgb.Blue,
                brightness = rgb.Brightness,
                speed = rgb.Speed
            };

            Console.WriteLine(JsonSerializer.Serialize(result, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to read RGB settings");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "set-rgb")
{
    await SetRgbAsync();
    return;

    async Task SetRgbAsync()
    {
        if (string.IsNullOrEmpty(device) || string.IsNullOrEmpty(mode))
        {
            Console.Error.WriteLine("usage: benchlab-cli set-rgb --device PATH --mode MODE [--red R] [--green G] [--blue B] [--brightness B] [--speed S]");
            Environment.Exit(2);
        }

        byte modeValue = mode.ToLower() switch
        {
            "off" => 0,
            "solid" => 1,
            "breathing" => 2,
            "cycle" => 3,
            "temperature" => 4,
            _ => throw new ArgumentException($"Invalid mode '{mode}'. Valid modes: off, solid, breathing, cycle, temperature")
        };

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            // Read current RGB settings to use as defaults
            var current = await protocol.ReadRgbAsync();

            var rgb = new RgbStruct
            {
                Mode = modeValue,
                Red = (byte)(red ?? current.Red),
                Green = (byte)(green ?? current.Green),
                Blue = (byte)(blue ?? current.Blue),
                Brightness = (byte)(brightness ?? current.Brightness),
                Speed = (byte)(speed ?? current.Speed)
            };

            var status = await protocol.WriteRgbAsync(rgb);
            Console.WriteLine(JsonSerializer.Serialize(new { status = status == 0 ? "ok" : $"error {status}", rgb = new { mode, red = rgb.Red, green = rgb.Green, blue = rgb.Blue, brightness = rgb.Brightness, speed = rgb.Speed } }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to set RGB");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "get-fan")
{
    await GetFanAsync();
    return;

    async Task GetFanAsync()
    {
        if (string.IsNullOrEmpty(device) || !fan.HasValue)
        {
            Console.Error.WriteLine("usage: benchlab-cli get-fan --device PATH --fan INDEX");
            Environment.Exit(2);
        }

        if (fan.Value < 0 || fan.Value > 8)
        {
            Console.Error.WriteLine("error: fan index must be 0-8");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var profile = await protocol.ReadFanProfileAsync((byte)fan.Value);
            var modeStr = profile.Mode == 0 ? "manual" : "auto";

            var result = new
            {
                device,
                fanIndex = fan.Value,
                mode = modeStr,
                manualDuty = profile.ManualDuty,
                tempThreshold = profile.TempThreshold / 100.0,
                minDuty = profile.MinDuty,
                maxDuty = profile.MaxDuty,
                sensorIndex = profile.SensorIndex
            };

            Console.WriteLine(JsonSerializer.Serialize(result, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to read fan profile");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "set-fan-manual")
{
    await SetFanManualAsync();
    return;

    async Task SetFanManualAsync()
    {
        if (string.IsNullOrEmpty(device) || !fan.HasValue || !duty.HasValue)
        {
            Console.Error.WriteLine("usage: benchlab-cli set-fan-manual --device PATH --fan INDEX --duty DUTY");
            Environment.Exit(2);
        }

        if (fan.Value < 0 || fan.Value > 8)
        {
            Console.Error.WriteLine("error: fan index must be 0-8");
            Environment.Exit(2);
        }

        if (duty.Value < 0 || duty.Value > 255)
        {
            Console.Error.WriteLine("error: duty must be 0-255");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var profile = new FanProfileStruct
            {
                Mode = 0, // Manual
                ManualDuty = (byte)duty.Value,
                TempThreshold = 0,
                MinDuty = 0,
                MaxDuty = 255,
                SensorIndex = 0,
                Reserved = 0
            };

            var status = await protocol.WriteFanProfileAsync((byte)fan.Value, profile);
            Console.WriteLine(JsonSerializer.Serialize(new { status = status == 0 ? "ok" : $"error {status}", fanIndex = fan.Value, mode = "manual", duty = duty.Value }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to set fan profile");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "set-fan-auto")
{
    await SetFanAutoAsync();
    return;

    async Task SetFanAutoAsync()
    {
        if (string.IsNullOrEmpty(device) || !fan.HasValue || !tempThreshold.HasValue || !minDuty.HasValue || !maxDuty.HasValue)
        {
            Console.Error.WriteLine("usage: benchlab-cli set-fan-auto --device PATH --fan INDEX --temp-threshold TEMP --min-duty MIN --max-duty MAX [--sensor SENSOR]");
            Environment.Exit(2);
        }

        if (fan.Value < 0 || fan.Value > 8)
        {
            Console.Error.WriteLine("error: fan index must be 0-8");
            Environment.Exit(2);
        }

        if (minDuty.Value < 0 || minDuty.Value > 255)
        {
            Console.Error.WriteLine("error: min-duty must be 0-255");
            Environment.Exit(2);
        }

        if (maxDuty.Value < 0 || maxDuty.Value > 255)
        {
            Console.Error.WriteLine("error: max-duty must be 0-255");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var profile = new FanProfileStruct
            {
                Mode = 1, // Auto
                ManualDuty = 0,
                TempThreshold = (short)(tempThreshold.Value * 100),
                MinDuty = (byte)minDuty.Value,
                MaxDuty = (byte)maxDuty.Value,
                SensorIndex = (byte)(sensor ?? 0),
                Reserved = 0
            };

            var status = await protocol.WriteFanProfileAsync((byte)fan.Value, profile);
            Console.WriteLine(JsonSerializer.Serialize(new { status = status == 0 ? "ok" : $"error {status}", fanIndex = fan.Value, mode = "auto", tempThreshold = tempThreshold.Value, minDuty = minDuty.Value, maxDuty = maxDuty.Value, sensorIndex = sensor ?? 0 }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to set fan profile");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "calibration")
{
    await CalibrationAsync();
    return;

    async Task CalibrationAsync()
    {
        if (string.IsNullOrEmpty(subcommand))
        {
            Console.Error.WriteLine("usage: benchlab-cli calibration {get|load|store|set} --device PATH");
            Environment.Exit(2);
        }

        if (subcommand == "get")
        {
            await CalibrationGetAsync();
        }
        else if (subcommand == "load")
        {
            await CalibrationLoadAsync();
        }
        else if (subcommand == "store")
        {
            await CalibrationStoreAsync();
        }
        else if (subcommand == "set")
        {
            await CalibrationSetAsync();
        }
        else
        {
            Console.Error.WriteLine($"unknown calibration subcommand: {subcommand}");
            Environment.Exit(2);
        }
    }

    async Task CalibrationGetAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli calibration get --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            unsafe
            {
                var cal = await protocol.ReadCalibrationAsync();
                var result = new
                {
                    device,
                    voltageOffsets = cal.VoltageOffsets,
                    voltageScales = cal.VoltageScales,
                    tempOffset = cal.TempOffset,
                    tempScale = cal.TempScale,
                    currentOffsets = cal.CurrentOffsets,
                    currentScales = cal.CurrentScales,
                    flags = cal.Flags
                };

                Console.WriteLine(JsonSerializer.Serialize(result, new JsonSerializerOptions { WriteIndented = true }));
            }
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to read calibration");
            Environment.Exit(1);
        }
    }

    async Task CalibrationLoadAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli calibration load --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var status = await protocol.LoadCalibrationAsync();
            var statusMsg = status switch
            {
                0 => "ok - calibration loaded from flash",
                1 => "warning - no calibration stored in flash",
                _ => $"error {status}"
            };

            Console.WriteLine(JsonSerializer.Serialize(new { status = statusMsg, statusCode = status }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to load calibration");
            Environment.Exit(1);
        }
    }

    async Task CalibrationStoreAsync()
    {
        if (string.IsNullOrEmpty(device))
        {
            Console.Error.WriteLine("usage: benchlab-cli calibration store --device PATH");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var status = await protocol.StoreCalibrationAsync();
            var statusMsg = status == 0 ? "ok - calibration saved to flash" : $"error {status} - flash write failed";

            Console.WriteLine(JsonSerializer.Serialize(new { status = statusMsg, statusCode = status }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to store calibration");
            Environment.Exit(1);
        }
    }

    async Task CalibrationSetAsync()
    {
        if (string.IsNullOrEmpty(device) || string.IsNullOrEmpty(json))
        {
            Console.Error.WriteLine("usage: benchlab-cli calibration set --device PATH --json FILE");
            Environment.Exit(2);
        }

        try
        {
            var jsonContent = File.ReadAllText(json);
            var cal = JsonSerializer.Deserialize<CalibrationStruct>(jsonContent);

            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var status = await protocol.WriteCalibrationAsync(cal);
            var statusMsg = status == 0 ? "ok - calibration applied (use 'store' to persist)" : $"error {status}";

            Console.WriteLine(JsonSerializer.Serialize(new { status = statusMsg, statusCode = status }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to set calibration");
            Environment.Exit(1);
        }
    }
}
else if (cmd == "action")
{
    await ActionAsync();
    return;

    async Task ActionAsync()
    {
        if (string.IsNullOrEmpty(device) || !actionId.HasValue)
        {
            Console.Error.WriteLine("usage: benchlab-cli action --device PATH --id ACTION_ID");
            Environment.Exit(2);
        }

        if (actionId.Value < 0 || actionId.Value > 255)
        {
            Console.Error.WriteLine("error: action ID must be 0-255");
            Environment.Exit(2);
        }

        try
        {
            using var sp = SerialPortAdapter.Open(device, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr: true, rts: true);
            var protocol = new BinaryProtocol(sp, TimeSpan.FromMilliseconds(timeoutMs), log);

            var status = await protocol.ExecuteActionAsync((byte)actionId.Value);
            var statusMsg = status == 0 ? "ok" : $"error {status}";

            Console.WriteLine(JsonSerializer.Serialize(new { status = statusMsg, statusCode = status, actionId = actionId.Value }, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch (Exception ex)
        {
            log.LogError(ex, "Failed to execute action");
            Environment.Exit(1);
        }
    }
}
else
{
    Help();
}
