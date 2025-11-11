using System.Text;
using BenchLab.Platform.Ports;
using BenchLab.Platform.Protocol;
using Microsoft.Extensions.Logging;

namespace BenchLab.Platform.Discovery;

/// <summary>
/// Proper BenchLab device handshake using the binary protocol.
/// This validates devices using UART_CMD_WELCOME and optionally UART_CMD_READ_VENDOR_DATA.
/// </summary>
public sealed class BenchlabHandshake : IBenchlabHandshake
{
    private readonly ILogger? _log;
    private readonly bool _validateVendorData;

    /// <summary>
    /// Creates a new BenchLab handshake validator.
    /// </summary>
    /// <param name="validateVendorData">If true, also validates vendor ID and product ID.</param>
    /// <param name="log">Optional logger.</param>
    public BenchlabHandshake(bool validateVendorData = true, ILogger? log = null)
    {
        _validateVendorData = validateVendorData;
        _log = log;
    }

    /// <summary>
    /// Probes a device to check if it's a valid BenchLab device.
    /// </summary>
    public ProbeResult Probe(string device, TimeSpan timeout)
    {
        ISerialPort? sp = null;
        try
        {
            // Open serial port with BenchLab settings
            sp = SerialPortAdapter.Open(
                device,
                baud: 115200,
                readTimeout: timeout,
                writeTimeout: timeout,
                dtr: true,
                rts: true
            );

            var protocol = new BinaryProtocol(sp, timeout, _log);

            // Step 1: Perform welcome handshake
            var welcomeSuccess = protocol.PerformHandshake();
            if (!welcomeSuccess)
            {
                _log?.LogDebug("Device {Device} failed welcome handshake", device);
                return ProbeResult.Failed(device, "Welcome handshake failed");
            }

            // Step 2: Optionally validate vendor data
            if (_validateVendorData)
            {
                try
                {
                    var vendorData = protocol.ReadVendorData();

                    if (!vendorData.IsValid)
                    {
                        _log?.LogDebug(
                            "Device {Device} has invalid vendor data (VID: 0x{VID:X2}, PID: 0x{PID:X2})",
                            device, vendorData.VendorId, vendorData.ProductId
                        );
                        return ProbeResult.Failed(
                            device,
                            $"Invalid vendor data: VID=0x{vendorData.VendorId:X2}, PID=0x{vendorData.ProductId:X2}"
                        );
                    }

                    _log?.LogInformation(
                        "BenchLab device detected: {Device} (FW v{Version})",
                        device, vendorData.FwVersion
                    );

                    return ProbeResult.Ok(
                        device,
                        $"BenchLab device (VID=0x{vendorData.VendorId:X2}, PID=0x{vendorData.ProductId:X2}, FW=v{vendorData.FwVersion})"
                    );
                }
                catch (Exception ex)
                {
                    _log?.LogWarning(ex, "Failed to read vendor data from {Device}", device);
                    // Still consider it valid if welcome succeeded, just return basic info
                    return ProbeResult.Ok(device, "BenchLab device (vendor data unavailable)");
                }
            }

            _log?.LogInformation("BenchLab device detected: {Device}", device);
            return ProbeResult.Ok(device, "BenchLab device (welcome validated)");
        }
        catch (UnauthorizedAccessException ex)
        {
            _log?.LogDebug(ex, "Access denied to {Device}", device);
            return ProbeResult.Failed(device, "Access denied");
        }
        catch (IOException ex)
        {
            _log?.LogDebug(ex, "I/O error probing {Device}", device);
            return ProbeResult.Failed(device, $"I/O error: {ex.Message}");
        }
        catch (TimeoutException ex)
        {
            _log?.LogDebug(ex, "Timeout probing {Device}", device);
            return ProbeResult.Failed(device, "Timeout");
        }
        catch (Exception ex)
        {
            _log?.LogWarning(ex, "Unexpected error probing {Device}", device);
            return ProbeResult.Failed(device, $"Error: {ex.Message}");
        }
        finally
        {
            sp?.Dispose();
        }
    }
}
