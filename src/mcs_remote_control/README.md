# MCS Remote Control

A modern web-based robot teleoperation interface for MCS ROS2 systems. This package provides a beautiful, responsive web application for controlling robots through ROS2 topics.

## Features

- 🎮 **Intuitive Control Interface**: Large, easy-to-use control buttons with visual feedback
- ⌨️ **Keyboard Support**: WASD keys and arrow keys for movement control
- 🎛️ **Speed Control**: Adjustable speed slider (10% - 100%)
- 🔄 **Auto-reconnection**: Automatically reconnects to ROS2 when connection is lost
- 📱 **Responsive Design**: Works on desktop, tablet, and mobile devices
- 🎨 **Modern UI**: Beautiful gradient design with smooth animations
- 🔌 **Real-time Status**: Visual connection status indicator

## Prerequisites

- Node.js (v16 or higher)
- npm (v8 or higher)
- ROS2 with `rosbridge_server` running
- WebSocket server running on `ws://localhost:9090`

## Installation

1. Navigate to the package directory:
   ```bash
   cd src/mcs_remote_control
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

## Usage

### Development Mode

Start the development server with hot reload:
```bash
npm run dev
```

This will start the development server on `http://localhost:3000` and automatically open your browser.

### Production Build

Build the application for production:
```bash
npm run build
```

This creates optimized files in the `dist/` directory.

### Serve Production Build

Serve the production build:
```bash
npm run serve
```

This serves the built application on `http://localhost:8080`.

## ROS2 Setup

### 1. Install rosbridge_server

```bash
sudo apt install ros-humble-rosbridge-server
```

### 2. Launch rosbridge_server

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Verify WebSocket Connection

The WebSocket server should be running on `ws://localhost:9090`. You can verify this by checking if the connection status indicator in the web interface shows "Connected".

## Control Interface

### Button Controls
- **↑ Forward**: Move robot forward
- **↓ Backward**: Move robot backward  
- **← Left**: Turn robot left
- **→ Right**: Turn robot right
- **⏹ Stop**: Stop all movement

### Keyboard Controls
- **W** or **↑**: Move forward
- **S** or **↓**: Move backward
- **A** or **←**: Turn left
- **D** or **→**: Turn right
- **Spacebar**: Stop

### Speed Control
Use the speed slider to adjust movement speed from 10% to 100%. The speed affects both linear and angular velocities.

## ROS2 Topics

The application publishes to the following ROS2 topic:

- **Topic**: `/cmd_vel`
- **Message Type**: `geometry_msgs/Twist`
- **Description**: Robot velocity commands

### Message Structure
```yaml
linear:
  x: 0.0  # Forward/backward velocity (m/s)
  y: 0.0  # Left/right velocity (m/s) 
  z: 0.0  # Up/down velocity (m/s)
angular:
  x: 0.0  # Roll angular velocity (rad/s)
  y: 0.0  # Pitch angular velocity (rad/s)
  z: 0.0  # Yaw angular velocity (rad/s)
```

## Configuration

### WebSocket URL
The default WebSocket URL is `ws://localhost:9090`. To change this, modify the `wsUrl` variable in `src/js/main.js`:

```javascript
const wsUrl = 'ws://your-ros2-host:9090';
```

### Speed Limits
Default speed values can be adjusted in `src/js/main.js`:

```javascript
// In handleControlAction method
case 'forward':
  linearX = 0.5 * this.speedMultiplier; // Adjust 0.5 to change max speed
  break;
```

## Development

### Project Structure
```
mcs_remote_control/
├── src/
│   ├── index.html          # Main HTML file
│   ├── styles/
│   │   └── main.css        # Stylesheets
│   └── js/
│       └── main.js         # Main JavaScript application
├── dist/                   # Production build output
├── package.json            # npm package configuration
├── vite.config.js          # Vite bundler configuration
└── README.md              # This file
```

### Adding Features

1. **New Controls**: Add buttons to `index.html` and handle them in `main.js`
2. **Additional Topics**: Create new ROSLIB.Topic instances in `main.js`
3. **Custom Styling**: Modify `main.css` for visual changes
4. **Configuration**: Add configuration options to the UI

### Building for Production

The application uses Vite for bundling. The build process:
1. Bundles all JavaScript modules
2. Optimizes and minifies code
3. Processes CSS with vendor prefixes
4. Generates static assets in `dist/` directory

## Troubleshooting

### Connection Issues
- Ensure `rosbridge_server` is running
- Check WebSocket URL in browser console
- Verify firewall settings allow WebSocket connections
- Check ROS2 network configuration

### Control Issues
- Verify `/cmd_vel` topic exists and is subscribed to
- Check message type compatibility
- Ensure robot driver is running and listening to `/cmd_vel`

### Build Issues
- Clear `node_modules` and reinstall: `rm -rf node_modules && npm install`
- Update Node.js to latest LTS version
- Check for conflicting global packages

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS2 and rosbridge_server documentation
3. Open an issue in the repository 