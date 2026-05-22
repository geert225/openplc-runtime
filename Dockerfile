# syntax=docker/dockerfile:1

FROM debian:bookworm-slim

# Runtime version baked in at build time (mirrors strucpp + editor —
# the value is the GitHub release tag, passed by .github/workflows/
# docker.yml).  Editors read this via /api/version to gate uploads
# (the v4.1.x runtime ships STruC++; older runtimes ship MatIEC).
ARG RUNTIME_VERSION=dev
ENV RUNTIME_VERSION=${RUNTIME_VERSION}

WORKDIR /workdir

# Copy source code
COPY . .

# Setup runtime directory and permissions
RUN mkdir -p /var/run/runtime && \
    chmod +x install.sh scripts/* start_openplc.sh

# Clean any existing build artifacts to ensure clean Docker build
RUN rm -rf build/ venvs/ .venv/ 2>/dev/null || true

# Run installation script
RUN ./install.sh

# Clean up apt cache to reduce image size (Docker-specific optimization)
RUN rm -rf /var/lib/apt/lists/*

# Expose webserver port
EXPOSE 8443

# Default execution - Start OpenPLC Runtime
CMD ["bash", "./start_openplc.sh"]
