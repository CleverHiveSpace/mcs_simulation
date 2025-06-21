#!/bin/bash

# MCS Remote Control Launch Script
# This script starts the development server for the robot control interface

echo "🤖 Starting MCS Remote Control..."
echo "📦 Installing dependencies if needed..."

# Check if node_modules exists, if not install dependencies
if [ ! -d "node_modules" ]; then
    echo "📥 Installing npm dependencies..."
    npm install
fi

echo "🚀 Starting development server..."
echo "🌐 The application will open at: http://localhost:3000"
echo "📱 Make sure rosbridge_server is running on ws://localhost:9090"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Start the development server
npm run dev
