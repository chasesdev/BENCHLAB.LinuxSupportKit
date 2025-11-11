using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading;
using BenchLab.Platform.Discovery;
using BenchLab.Platform.Ports;
using Microsoft.Extensions.Logging;

var logFactory = LoggerFactory.Create(builder => builder.AddConsole());
var log = logFactory.CreateLogger("benchlab-cli");

static void Help()
{
    Console.WriteLine("benchlab-cli commands:");
    Console.WriteLine("  list [--timeout ms]");
    Console.WriteLine("  stream [--device PATH] [--timeout ms] [--raw]");
    Console.WriteLine("  write --device PATH --text "STRING"");
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
    var d = new PortDiscovery(new SimpleHandshake(ping: null), log);
    var results = d.Discover(TimeSpan.FromMilliseconds(timeoutMs)).ToArray();
    Console.WriteLine(JsonSerializer.Serialize(results, new JsonSerializerOptions { WriteIndented = true }));
    return;
}
else if (cmd == "stream")
{
    var d = new PortDiscovery(new SimpleHandshake(ping: null), log);
    var results = d.Discover(TimeSpan.FromMilliseconds(timeoutMs)).ToArray();
    var target = device ?? results.FirstOrDefault(r => r.IsBenchlab)?.Device ?? results.FirstOrDefault()?.Device;
    if (target is null) { Console.Error.WriteLine("no device found"); Environment.Exit(2); }
    log.LogInformation("Streaming from {Device}", target);

    using var sp = SerialPortAdapter.Open(target, 115200, TimeSpan.FromMilliseconds(timeoutMs), TimeSpan.FromMilliseconds(timeoutMs), dtr:true, rts:true);
    var buf = new byte[256];
    while (true)
    {
        try
        {
            if (sp.BytesToRead > 0)
            {
                var n = sp.Read(buf, 0, Math.Min(buf.Length, sp.BytesToRead));
                if (n > 0)
                {
                    if (raw)
                    {
                        // emit hex NDJSON
                        var hex = BitConverter.ToString(buf, 0, n).Replace("-", " ");
                        Console.WriteLine($"{{"device":"{target}","hex":"{hex}","ts":"{DateTime.UtcNow:o}"}}");
                    }
                    else
                    {
                        var s = Encoding.UTF8.GetString(buf, 0, n);
                        foreach (var line in s.Split('\n'))
                        {
                            if (string.IsNullOrWhiteSpace(line)) continue;
                            Console.WriteLine($"{{"device":"{target}","line":{JsonSerializer.Serialize(line)},"ts":"{DateTime.UtcNow:o}"}}");
                        }
                    }
                }
            }
            Thread.Sleep(10);
        }
        catch (Exception ex)
        {
            log.LogError(ex, "stream error");
            Thread.Sleep(200);
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
