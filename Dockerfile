# Multi-stage Dockerfile for BenchLab HTTP Service
FROM mcr.microsoft.com/dotnet/sdk:8.0 AS build
WORKDIR /src

# Copy project files
COPY src/BenchLab.Platform/BenchLab.Platform.csproj src/BenchLab.Platform/
COPY src/BenchLab.Service/BenchLab.Service.csproj src/BenchLab.Service/
RUN dotnet restore src/BenchLab.Service/BenchLab.Service.csproj

# Copy source code
COPY src/ src/

# Build and publish
RUN dotnet publish src/BenchLab.Service/BenchLab.Service.csproj \
    -c Release \
    -o /app/publish \
    --no-restore \
    -p:PublishSingleFile=false \
    -p:PublishTrimmed=true

# Runtime stage
FROM mcr.microsoft.com/dotnet/aspnet:8.0-jammy
WORKDIR /app

# Install udev rules and required packages
RUN apt-get update && apt-get install -y \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Copy published application
COPY --from=build /app/publish .

# Copy udev rules
COPY udev/99-benchlab.rules /etc/udev/rules.d/

# Create benchlab user and add to dialout group
RUN useradd --system --no-create-home -G dialout benchlab

# Set environment variables
ENV ASPNETCORE_URLS=http://+:8080 \
    BENCHLAB_BIND_ADDRESS=http://0.0.0.0:8080 \
    DOTNET_RUNNING_IN_CONTAINER=true \
    DOTNET_SYSTEM_GLOBALIZATION_INVARIANT=true

# Expose port
EXPOSE 8080

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8080/health || exit 1

# Run as benchlab user
USER benchlab

ENTRYPOINT ["./benchlabd"]
