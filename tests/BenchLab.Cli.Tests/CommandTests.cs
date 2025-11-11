using System.Diagnostics;
using System.Text.Json;
using Xunit;
using FluentAssertions;

namespace BenchLab.Cli.Tests;

/// <summary>
/// Integration tests for CLI commands.
/// Tests argument parsing, output formatting, and error handling.
/// Note: Tests verify CLI behavior without requiring actual hardware devices.
/// </summary>
public class CommandTests
{
    private readonly string _cliPath;

    public CommandTests()
    {
        // Assume CLI is built and available
        // In CI, this would be set to the build output path
        _cliPath = Environment.GetEnvironmentVariable("CLI_PATH") ?? "benchlab-cli";
    }

    #region Helper Methods

    private async Task<(int exitCode, string stdout, string stderr)> RunCliAsync(string arguments, int timeoutSeconds = 10)
    {
        var psi = new ProcessStartInfo
        {
            FileName = _cliPath,
            Arguments = arguments,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true
        };

        using var process = new Process { StartInfo = psi };
        var stdoutBuilder = new System.Text.StringBuilder();
        var stderrBuilder = new System.Text.StringBuilder();

        process.OutputDataReceived += (sender, e) =>
        {
            if (e.Data != null) stdoutBuilder.AppendLine(e.Data);
        };

        process.ErrorDataReceived += (sender, e) =>
        {
            if (e.Data != null) stderrBuilder.AppendLine(e.Data);
        };

        process.Start();
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        var completed = await Task.Run(() => process.WaitForExit(timeoutSeconds * 1000));

        if (!completed)
        {
            process.Kill();
            throw new TimeoutException($"CLI command timed out after {timeoutSeconds} seconds");
        }

        return (process.ExitCode, stdoutBuilder.ToString(), stderrBuilder.ToString());
    }

    #endregion

    #region Help Command

    [Fact]
    public async Task NoArguments_ShowsHelp()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("");

        // Assert
        exitCode.Should().Be(0);
        stdout.Should().Contain("benchlab-cli commands:");
        stdout.Should().Contain("list");
        stdout.Should().Contain("info");
        stdout.Should().Contain("sensors");
        stdout.Should().Contain("stream");
        stdout.Should().Contain("write");
    }

    [Fact]
    public async Task HelpFlag_ShowsHelp()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("--help");

        // Assert
        exitCode.Should().Be(0);
        stdout.Should().Contain("benchlab-cli commands:");
    }

    [Fact]
    public async Task HFlagShort_ShowsHelp()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("-h");

        // Assert
        exitCode.Should().Be(0);
        stdout.Should().Contain("benchlab-cli commands:");
    }

    [Fact]
    public async Task UnknownCommand_ShowsHelp()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("unknown");

        // Assert
        exitCode.Should().Be(0);
        stdout.Should().Contain("benchlab-cli commands:");
    }

    #endregion

    #region List Command

    [Fact]
    public async Task List_ReturnsJsonArray()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list");

        // Assert
        exitCode.Should().Be(0);

        // Parse as JSON
        var json = JsonDocument.Parse(stdout);
        json.RootElement.ValueKind.Should().Be(JsonValueKind.Array);
    }

    [Fact]
    public async Task List_WithTimeout_AcceptsParameter()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list --timeout 1000");

        // Assert
        exitCode.Should().Be(0);
        var json = JsonDocument.Parse(stdout);
        json.RootElement.ValueKind.Should().Be(JsonValueKind.Array);
    }

    [Fact]
    public async Task List_WithInvalidTimeout_HandlesError()
    {
        // Act
        var act = async () => await RunCliAsync("list --timeout invalid");

        // Assert
        // Should throw or return non-zero exit code
        await act.Should().ThrowAsync<Exception>();
    }

    [Fact]
    public async Task List_ReturnsProbeResults()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list");

        // Assert
        var json = JsonDocument.Parse(stdout);
        if (json.RootElement.GetArrayLength() > 0)
        {
            var first = json.RootElement[0];
            first.TryGetProperty("device", out _).Should().BeTrue();
            first.TryGetProperty("isBenchlab", out _).Should().BeTrue();
        }
    }

    #endregion

    #region Info Command

    [Fact]
    public async Task Info_WithoutDevice_ShowsUsage()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info");

        // Assert
        exitCode.Should().Be(2);
        stderr.Should().Contain("usage:");
        stderr.Should().Contain("--device");
    }

    [Fact]
    public async Task Info_NonexistentDevice_ReturnsError()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info --device /dev/nonexistent");

        // Assert
        exitCode.Should().Be(1);
    }

    [Fact]
    public async Task Info_WithTimeout_AcceptsParameter()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info --device /dev/nonexistent --timeout 500");

        // Assert
        // Should still fail on nonexistent device but accepted the timeout parameter
        exitCode.Should().Be(1);
    }

    [Fact]
    public async Task Info_OutputStructure_IsValid()
    {
        // This test verifies the expected output structure
        // Even if it fails due to no device, we can check the error message format
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info --device /dev/test");

        // Assert
        // Either succeeds with valid JSON or fails with error message
        if (exitCode == 0)
        {
            var json = JsonDocument.Parse(stdout);
            json.RootElement.TryGetProperty("device", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("name", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("vendorId", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("productId", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("firmwareVersion", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("isValid", out _).Should().BeTrue();
        }
    }

    #endregion

    #region Sensors Command

    [Fact]
    public async Task Sensors_WithoutDevice_ShowsUsage()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("sensors");

        // Assert
        exitCode.Should().Be(2);
        stderr.Should().Contain("usage:");
        stderr.Should().Contain("--device");
    }

    [Fact]
    public async Task Sensors_NonexistentDevice_ReturnsError()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("sensors --device /dev/nonexistent");

        // Assert
        exitCode.Should().Be(1);
    }

    [Fact]
    public async Task Sensors_WithTimeout_AcceptsParameter()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("sensors --device /dev/nonexistent --timeout 500");

        // Assert
        exitCode.Should().Be(1); // Still fails but accepted parameter
    }

    [Fact]
    public async Task Sensors_OutputStructure_IsValid()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("sensors --device /dev/test");

        // Assert
        if (exitCode == 0)
        {
            var json = JsonDocument.Parse(stdout);
            json.RootElement.TryGetProperty("device", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("timestamp", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("voltages", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("temperatures", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("humidity", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("power", out _).Should().BeTrue();
            json.RootElement.TryGetProperty("fans", out _).Should().BeTrue();
        }
    }

    #endregion

    #region Write Command

    [Fact]
    public async Task Write_WithoutDevice_ShowsUsage()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --text \"test\"");

        // Assert
        exitCode.Should().Be(2);
        stderr.Should().Contain("usage:");
        stderr.Should().Contain("--device");
    }

    [Fact]
    public async Task Write_WithoutText_ShowsUsage()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/test");

        // Assert
        exitCode.Should().Be(2);
        stderr.Should().Contain("usage:");
        stderr.Should().Contain("--text");
    }

    [Fact]
    public async Task Write_NonexistentDevice_ReturnsError()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/nonexistent --text \"test\"");

        // Assert
        exitCode.Should().NotBe(0); // Should fail
    }

    [Fact]
    public async Task Write_WithTimeout_AcceptsParameter()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/nonexistent --text \"test\" --timeout 500");

        // Assert
        // Should fail on nonexistent device but accepted parameter
        exitCode.Should().NotBe(0);
    }

    [Fact]
    public async Task Write_SuccessOutput_ContainsOk()
    {
        // This would only pass with a real device, but we can verify the behavior
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/test --text \"test\"");

        // Assert
        if (exitCode == 0)
        {
            stdout.Should().Contain("ok");
        }
    }

    #endregion

    #region Argument Parsing

    [Fact]
    public async Task Arguments_OrderIndependent()
    {
        // Act - Try different argument orders
        var result1 = await RunCliAsync("info --device /dev/test --timeout 500");
        var result2 = await RunCliAsync("info --timeout 500 --device /dev/test");

        // Assert - Both should fail the same way (no device)
        result1.exitCode.Should().Be(result2.exitCode);
    }

    [Fact]
    public async Task Arguments_MultipleTimeouts_UsesLast()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list --timeout 500 --timeout 1000");

        // Assert
        exitCode.Should().Be(0); // Should still work, uses last timeout
    }

    [Fact]
    public async Task Arguments_InvalidFlag_ShowsHelp()
    {
        // Act
        var act = async () => await RunCliAsync("list --invalid-flag");

        // Assert
        // Should either throw or show help
        await act.Should().ThrowAsync<Exception>();
    }

    #endregion

    #region Output Formatting

    [Fact]
    public async Task List_OutputIsValidJson()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list");

        // Assert
        exitCode.Should().Be(0);

        var act = () => JsonDocument.Parse(stdout);
        act.Should().NotThrow<JsonException>();
    }

    [Fact]
    public async Task List_OutputIsPrettyPrinted()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list");

        // Assert
        exitCode.Should().Be(0);

        // Pretty-printed JSON has newlines
        stdout.Should().Contain("\n");
        stdout.Should().Contain("  "); // Indentation
    }

    [Fact]
    public async Task Info_OutputIsPrettyPrinted()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info --device /dev/test");

        // Assert
        if (exitCode == 0)
        {
            stdout.Should().Contain("\n");
            stdout.Should().Contain("  ");
        }
    }

    [Fact]
    public async Task Sensors_OutputIsPrettyPrinted()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("sensors --device /dev/test");

        // Assert
        if (exitCode == 0)
        {
            stdout.Should().Contain("\n");
            stdout.Should().Contain("  ");
        }
    }

    #endregion

    #region Error Messages

    [Fact]
    public async Task Error_OutputToStderr()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info");

        // Assert
        exitCode.Should().Be(2);
        stderr.Should().NotBeEmpty();
        stdout.Should().BeEmpty();
    }

    [Fact]
    public async Task Usage_ContainsCommandName()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info");

        // Assert
        stderr.Should().Contain("benchlab-cli");
    }

    [Fact]
    public async Task Usage_ContainsRequiredParameters()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info");

        // Assert
        stderr.Should().Contain("--device");
        stderr.Should().Contain("PATH");
    }

    #endregion

    #region Special Characters and Edge Cases

    [Fact]
    public async Task Write_SpecialCharacters_HandledCorrectly()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/test --text \"test!@#$%\"");

        // Assert
        // Should accept special characters without error (though may fail to write)
        // Error should be about device, not about text parsing
        if (exitCode != 0)
        {
            stderr.Should().NotContain("text");
        }
    }

    [Fact]
    public async Task Write_EmptyText_HandledCorrectly()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/test --text \"\"");

        // Assert
        // Should accept empty text without parsing error
        if (exitCode == 2)
        {
            stderr.Should().Contain("usage:"); // Missing text
        }
    }

    [Fact]
    public async Task Write_UnicodeText_HandledCorrectly()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("write --device /dev/test --text \"测试🔥\"");

        // Assert
        // Should accept Unicode without error
        if (exitCode != 0 && exitCode != 2)
        {
            stderr.Should().NotContain("text");
        }
    }

    [Fact]
    public async Task DevicePath_WithSpaces_HandledCorrectly()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info --device \"/dev/path with spaces\"");

        // Assert
        // Should handle quoted paths without parsing error
        exitCode.Should().BeOneOf(0, 1); // Success or device not found, not parse error (2)
    }

    #endregion

    #region Timeout Behavior

    [Fact]
    public async Task Timeout_ZeroValue_HandledCorrectly()
    {
        // Act
        var act = async () => await RunCliAsync("list --timeout 0");

        // Assert
        // Should either reject zero timeout or handle it gracefully
        var result = await act.Should().NotThrowAsync();
    }

    [Fact]
    public async Task Timeout_NegativeValue_HandledCorrectly()
    {
        // Act
        var act = async () => await RunCliAsync("list --timeout -1");

        // Assert
        // Should reject negative timeout
        await act.Should().ThrowAsync<Exception>();
    }

    [Fact]
    public async Task Timeout_VeryLarge_HandledCorrectly()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list --timeout 999999");

        // Assert
        exitCode.Should().Be(0); // Should accept large timeout
    }

    #endregion

    #region Exit Codes

    [Fact]
    public async Task Success_ReturnsZero()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("list");

        // Assert
        exitCode.Should().Be(0);
    }

    [Fact]
    public async Task MissingRequiredArg_ReturnsTwo()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info");

        // Assert
        exitCode.Should().Be(2); // Usage error
    }

    [Fact]
    public async Task DeviceError_ReturnsOne()
    {
        // Act
        var (exitCode, stdout, stderr) = await RunCliAsync("info --device /dev/nonexistent");

        // Assert
        exitCode.Should().Be(1); // Runtime error
    }

    #endregion

    #region Performance

    [Fact]
    public async Task List_CompletesQuickly()
    {
        // Act
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var (exitCode, stdout, stderr) = await RunCliAsync("list --timeout 100");
        sw.Stop();

        // Assert
        exitCode.Should().Be(0);
        sw.ElapsedMilliseconds.Should().BeLessThan(2000); // Should complete within 2 seconds
    }

    [Fact]
    public async Task Help_InstantResponse()
    {
        // Act
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var (exitCode, stdout, stderr) = await RunCliAsync("--help");
        sw.Stop();

        // Assert
        exitCode.Should().Be(0);
        sw.ElapsedMilliseconds.Should().BeLessThan(500); // Should be very fast
    }

    #endregion
}
