# Base image
FROM python:3.10-slim

# Set environment variables to avoid interactive prompts during builds
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

# Update and install required system packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-dev \
    libi2c-dev \
    git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy the requirements file to the container
COPY requirements.txt /app/

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Clone and install the Adafruit library from GitHub
RUN git clone https://github.com/adafruit/Adafruit_Python_DHT.git /tmp/Adafruit_Python_DHT && \
    cd /tmp/Adafruit_Python_DHT && \
    python setup.py install --force-pi && \
    cd / && rm -rf /tmp/Adafruit_Python_DHT

# Clone and install the picamera library (if needed, replace with correct GitHub repo if applicable)
RUN git clone https://github.com/waveform80/picamera.git /tmp/picamera && \
    cd /tmp/picamera && \
    python setup.py install && \
    cd / && rm -rf /tmp/picamera

# Copy the application files into the container
COPY /app/agmad_family_farm /app/agmad_family_farm

# Set the command to run both Python scripts in parallel
CMD ["sh", "-c", "python /app/agmad_family_farm/main.py"]
